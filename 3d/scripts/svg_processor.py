#   Copyright 2015-2016 Scott Bezek and the splitflap contributors
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.

from collections import defaultdict

from svg.path import (
    Path,
    Line,
    Move,
    Close,
    parse_path,
)
from xml.dom import minidom

eps = 0.001


def _get_slope_intersect(p1, p2):
    if abs(p1.real - p2.real) < eps:
        # Vertical; no slope and return x-intercept
        return None, p1.real
    m = (p2.imag - p1.imag) / (p2.real - p1.real)
    b1 = p1.imag - m * p1.real
    b2 = p2.imag - m * p2.real
    assert abs(b1 - b2) < eps
    return m, b1


def _lines_are_collinear(line1, line2):
    eq1 = _get_slope_intersect(line1.start, line1.end)
    eq2 = _get_slope_intersect(line2.start, line2.end)

    same_slope = ((eq1[0] is None and eq2[0] is None)
                  or (eq1[0] is not None and eq2[0] is not None and abs(eq1[0] - eq2[0]) < eps))
    same_intercept = abs(eq1[1] - eq2[1]) < eps

    return same_slope and same_intercept


class SvgProcessor(object):
    """
    Processes SVG files generated by OpenSCAD to prepare for laser cutting
    """

    def __init__(self, input_file):
        self.dom = minidom.parse(input_file)
        self.svg_node = self.dom.documentElement

    def set_dimensions(self, width, height):
        self.svg_node.attributes['width'].value = width
        self.svg_node.attributes['height'].value = height

    def set_viewbox(self, min_x, min_y, width, height):
        view_str = "{:.0f} {:.0f} {:.0f} {:.0f}".format(min_x, min_y, width, height)
        self.svg_node.attributes['viewBox'].value = view_str

    def get_viewbox(self):
        vb = self.svg_node.attributes['viewBox'].value.replace(',', '').split(' ')
        vb = [float(x) for x in vb]
        return tuple(vb)

    def apply_laser_cut_style(self):
        # Set fill and stroke for laser cutting
        for path in self.svg_node.getElementsByTagName('path'):
                SvgProcessor._apply_attributes(path, {
                    'fill': 'none',
                    'stroke': '#0000ff',
                    'stroke-width': '0.1',
                })

    def apply_laser_etch_style(self):
        # Set fill and stroke for laser etching
        for path in self.svg_node.getElementsByTagName('path'):
                SvgProcessor._apply_attributes(path, {
                    'fill': '#000000',
                    'stroke': 'none',
                })

    def apply_raster_render_style(self):
        # Set fill and stroke for rasterized rendering
        for path in self.svg_node.getElementsByTagName('path'):
                SvgProcessor._apply_attributes(path, {
                    'fill': 'none',
                    'stroke': '#000000',
                    'stroke-width': '0.2',
                })

    def import_paths(self, from_svg_processor):
        for path in from_svg_processor.svg_node.getElementsByTagName('path'):
            output_node = self.dom.importNode(path, True)
            self.svg_node.appendChild(output_node)

        vb = self.merge_viewbox(from_svg_processor.get_viewbox())
        self.set_viewbox(*vb)
        dimm = "{:.0f}mm"
        self.set_dimensions(dimm.format(vb[2]), dimm.format(vb[3]))


    def merge_viewbox(self, vb1):
        """
        Takes a new SVG viewbox and combines it with the existing viewbox
        to create a new viewbox enclosing both of them.

        Returns a tuple with the min-x, min-y, width, and height
        """

        def get_max(vb, ax):
            return vb[ax] + vb[ax+2]  # min + size

        vb2 = self.get_viewbox()

        mins, maxes = [], []
        for xy in range(2):
            ax_max = [get_max(vb1, xy), get_max(vb2, xy)]
            mins.append(vb1[xy] if vb1[xy] < vb2[xy] else vb2[xy])
            maxes.append(ax_max[0] if ax_max[0] > ax_max[1] else ax_max[1])

        X, Y = 0, 1
        return (mins[X], mins[Y], maxes[X] - mins[X], maxes[Y] - mins[Y])

    def remove_redundant_lines(self):
        lines_bucketed_by_slope_intersect = defaultdict(list)
        paths = self.svg_node.getElementsByTagName('path')
        overall_index = 0
        for path_index, path in enumerate(paths):
            path_text = path.attributes['d'].value
            path_obj = parse_path(path_text)
            for line_index, line in enumerate(path_obj):
                slope, intersect = _get_slope_intersect(line.start, line.end)
                # Moves don't draw anything by themselves, but they do set the
                # target for subsequent closes, so they should not be removed.
                if not isinstance(line, Move):
                    # TODO: float inaccuracy and rounding may cause collinear lines to end up in separate buckets in rare
                    # cases, so this is not quite correct. Would be better to put lines into *2* nearest buckets in each
                    # dimension to avoid edge cases.
                    if slope is not None:
                        slope = round(slope, ndigits=3)
                    intersect = round(intersect, ndigits=3)
                    lines_bucketed_by_slope_intersect[(slope, intersect)].append({
                        'overall_index': overall_index,
                        'path_index': path_index,
                        'line_index': line_index,
                        'line': line,
                    })
                overall_index += 1

        to_remove = {}
        to_update = {}
        for _, lines in list(lines_bucketed_by_slope_intersect.items()):
            for i in range(20):
                if SvgProcessor._pairwise_overlap_check(lines, to_update, to_remove):
                    print('Re-running pairwise overlap check because of updated/merged line')
                    continue
                break
            else:
                raise Exception(
                    'Exceeded the max number of pairwise overlap check passes. Something is probably wrong.'
                )

        # Reconstruct the paths, but excluding/updating the lines we just identified
        i = 0
        removed = 0
        removed_length = 0
        kept = 0
        kept_length = 0
        for path_index, path in enumerate(paths):
            path_text = path.attributes['d'].value
            path_obj = parse_path(path_text)

            filtered_path = Path()

            for line_index, line in enumerate(path_obj):
                if i in to_remove:
                    assert path_index == to_remove[i][0]
                    assert line_index == to_remove[i][1]
                    removed += 1
                    removed_length += line.length()
                elif i in to_update:
                    assert path_index == to_update[i][0]
                    assert line_index == to_update[i][1]
                    replacement_line = to_update[i][2]

                    filtered_path.append(replacement_line)
                    kept += 1
                    kept_length += replacement_line.length()
                elif isinstance(line, Close):
                    # Replace the close with a line, because if we removed all
                    # or part of the previous line in this path, a close will
                    # not work as expected.
                    new_line = Line(line.start, line.end)
                    filtered_path.append(new_line)
                    kept += 1
                    kept_length += new_line.length()
                else:
                    filtered_path.append(line)
                    kept += 1
                    kept_length += line.length()
                i += 1

            # Update the path data with the filtered path data
            path.attributes['d'] = filtered_path.d()

        print('Removed {} lines ({} length) and kept {} lines ({} length)'.format(
            removed,
            removed_length,
            kept,
            kept_length,
        ))

        return [to_remove[k][2] for k in to_remove], [to_update[k][2] for k in to_update]

    @staticmethod
    def _pairwise_overlap_check(lines, to_update, to_remove):
        """Naive N^2 search for overlapping lines within a slope-intersect bucket.

        Adds lines to the to_remove dictionary when they are fully redundant, and adds updated line info to the
        to_update dictionary if a line needs to be lengthened to simplify partially overlapping lines.

        Returns True if a line update was produced (which means another pass of overlap-checking is required with the
        updated line info.
        """
        for i in range(len(lines)):
            if lines[i]['overall_index'] in to_remove:
                continue
            line1 = lines[i]['line']
            for j in range(i + 1, len(lines)):
                if lines[j]['overall_index'] in to_remove:
                    continue
                line2 = lines[j]['line']

                if _lines_are_collinear(line1, line2):
                    # Check for overlap using the min/max x and y values of the lines
                    l1x1 = min(line1.start.real, line1.end.real)
                    l1x2 = max(line1.start.real, line1.end.real)
                    l1y1 = min(line1.start.imag, line1.end.imag)
                    l1y2 = max(line1.start.imag, line1.end.imag)

                    l2x1 = min(line2.start.real, line2.end.real)
                    l2x2 = max(line2.start.real, line2.end.real)
                    l2y1 = min(line2.start.imag, line2.end.imag)
                    l2y2 = max(line2.start.imag, line2.end.imag)

                    if l1x1 <= l2x1 + eps and l1x2 + eps >= l2x2 and l1y1 <= l2y1 + eps and l1y2 + eps >= l2y2:
                        # Line 1 is bigger, fully contains line 2
                        assert line1.length() + eps >= line2.length()
                        to_remove[lines[j]['overall_index']] = (lines[j]['path_index'], lines[j]['line_index'], line2)
                    elif l1x1 + eps >= l2x1 and l1x2 <= l2x2 + eps and l1y1 + eps >= l2y1 and l1y2 <= l2y2 + eps:
                        # Line 2 is bigger, fully contains line 1
                        assert line2.length() + eps >= line1.length()
                        to_remove[lines[i]['overall_index']] = (lines[i]['path_index'], lines[i]['line_index'], line1)

                    # Check for partial overlap, i.e. one point of line 2 is between points of line 1 or vice versa
                    # To avoid cases with 2 line segments end-to-end, we check for point containment with either
                    # X inclusive OR Y inclusive, but not both (which would mean they share an endpoint and therefore
                    # must be end-to-end rather than overlapping since we already covered the fully-contained cases
                    # above)
                    elif (
                        (l1x1 <= l2x1 + eps and l2x1 <= l1x2 + eps and l1y1 + eps < l2y1 and l2y1 + eps < l1y2) or
                        (l1x1 + eps < l2x1 and l2x1 + eps < l1x2 and l1y1 <= l2y1 + eps and l2y1 <= l1y2 + eps) or
                        (l1x1 <= l2x2 + eps and l2x2 <= l1x2 + eps and l1y1 + eps < l2y2 and l2y2 + eps < l1y2) or
                        (l1x1 + eps < l2x2 and l2x2 + eps < l1x2 and l1y1 <= l2y2 + eps and l2y2 <= l1y2 + eps)
                    ):
                        print('Partial overlap of these lines:\n  {!r}\n  {!r}'.format(line1, line2))

                        # Arbitrarily pick line1 to remove, and update line2 to cover the full length
                        to_remove[lines[i]['overall_index']] = (
                            lines[i]['path_index'],
                            lines[i]['line_index'],
                            line1,
                        )
                        if lines[i]['overall_index'] in to_update:
                            # In case we're now deleting a line that was previously updated, remove it from
                            # to_update to be safe
                            del to_update[lines[i]['overall_index']]

                        # To form a line that covers the full length, try all pairs of points and select the pair
                        # that produces the longest length.
                        #
                        # Simply sorting the points as x,y tuples and choosing the first/last wouldn't work because
                        # of possible floating point error: if the 2 lines have the exact same x coordinate then the
                        # sort will fall back to sorting on y and work as expected, but if the 2 lines have the
                        # "same" x coordinate but one is actually a miniscule amount smaller, that x difference will
                        # take precedence in the sort, potentially resulting in the wrong endpoints being selected.
                        points = [
                            line1.start,
                            line1.end,
                            line2.start,
                            line2.end,
                        ]

                        longest_line = line1
                        for x in range(len(points)):
                            for y in range(x + 1, len(points)):
                                new_line = Line(points[x], points[y])
                                if new_line.length() > longest_line.length():
                                    longest_line = new_line

                        # Update the original line's values (needed for subsequent comparisons of collinear lines)
                        # and log an entry in to_update for the final SVG path generation.
                        line2.start = longest_line.start
                        line2.end = longest_line.end
                        to_update[lines[j]['overall_index']] = (
                            lines[j]['path_index'],
                            lines[j]['line_index'],
                            line2,
                        )
                        print('  -- merged into a single line: {!r}'.format(line2))
                        return True
        return False

    def add_highlight_lines(self, lines, color):
        for line in lines:
            new_path_node = self.dom.createElement("path")

            new_path_node.setAttribute('d', Path(line).d())
            new_path_node.setAttribute('fill', 'none')
            new_path_node.setAttribute('stroke', color)
            new_path_node.setAttribute('stroke-width', '1')
            new_path_node.setAttribute('stroke-opacity', '.45')

            self.svg_node.appendChild(new_path_node)

    def write(self, filename):
        with open(filename, 'w') as output_file:
            self.svg_node.writexml(output_file)

    @staticmethod
    def _apply_attributes(node, values):
        for (k, v) in list(values.items()):
            node.attributes[k] = v
