from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
# noinspection PyUnresolvedReferences,PyCompatibility
from builtins import *

from typing import Any, Set, Dict, Optional, Union, List, Tuple

from bag.layout.routing import TrackID, WireArray
from pybag.enum import RoundMode, MinLenMode, Orient2D, Direction

from bag.layout.template import TemplateDB
from bag.layout.util import BBox
from bag.util.math import HalfInt

from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase, TrackManager
from bag.layout.template import TemplateBase
import math


def via_maker(template, l1, l2, object, coordx, coordy, width):
    base = object
    layers = []
    width = [0]*l1+ width
    for i in range(l1, l2):
        if (i + 1) % 2 == 0:
            out_track = TrackID(i + 1, template.grid.coord_to_nearest_track(i + 1, coordy, half_track=True, mode=0,
                                                                            unit_mode=True), width=width[i])
        else:
            out_track = TrackID(i + 1, template.grid.coord_to_nearest_track(i + 1, coordx, half_track=True, mode=0,
                                                                            unit_mode=True), width=width[i])
        layers.append(template.connect_to_tracks([base], out_track, min_len_mode=0))
        base = layers[-1]

    return layers


def get_mos_conn_layer(self):
    return self.grid.tech_info.tech_params['layout']['mos_tech_class'].get_mos_conn_layer()


def max_conn_wires(self, tr_manager, wire_type, wire_list, start_coord=None, end_coord=None):
    max_coord, min_coord = 0, math.inf
    for w in wire_list:
        max_coord = max_coord if max_coord > w.upper else w.upper
        min_coord = min_coord if min_coord < w.lower else w.lower

    start_coord = start_coord if start_coord is not None else min_coord
    end_coord = end_coord if end_coord is not None else max_coord
    if end_coord < start_coord:
        raise ValueError("[Util Error:] End points smaller than start point, please check")
    conn_layer = wire_list[0].layer_id+1
    conn_w = tr_manager.get_width(conn_layer, wire_type)
    cur_tidx = self.grid.coord_to_track(conn_layer, start_coord, mode=RoundMode.NEAREST)
    res_wire_list = []
    while self.grid.track_to_coord(conn_layer, cur_tidx) <= end_coord:
        res_wire_list.append(self.connect_to_tracks(wire_list, TrackID(conn_layer, cur_tidx, conn_w)))
        cur_tidx = tr_manager.get_next_track(conn_layer, cur_tidx, wire_type, wire_type)
    if len(res_wire_list) < 1:
        raise ValueError("[Util Error:] Targeted connection have no effect")
    return res_wire_list


def round_to_blk_pitch(x_loc, y_loc, blk_w, blk_h):
    return (math.ceil(x_loc / blk_w) * blk_w, math.ceil(y_loc / blk_h) * blk_h)


def draw_stack_wire(self: TemplateBase, wire: WireArray, top_layid: int, x0: Optional[Union[int, float]] = None,
                    y0: Optional[Union[int, float]] = None, x1: Optional[Union[int, float]] = None,
                    y1: Optional[Union[int, float]] = None, tr_list: Optional[List[Union[int, float]]] = None,
                    sp_list: Optional[List[Union[HalfInt, int]]] = None, max_mode: bool = True,
                    min_len_mode: MinLenMode = MinLenMode.NONE,
                    mode: int = 1, sep_margin: bool = False,
                    sp_margin_list: Optional[List[Union[HalfInt, int]]] = None) -> List[List[WireArray]]:
    """
        create WireArray from bot_layid to top_layid-1 within the given coordinates
    Parameters
    ----------
    self: TemplateBase
        template database.
    wire: WireArray
        wire to be connected.
    top_layid: Int
        top layer id.
    x0: Union[Int, float]
        bottom left, x coordinate.
    y0: Union[Int, float]
        bottom left, y coordinate.
    x1: Union[Int, float]
        top right, x coordinate.
    y1: Union[Int, float]
        top right, y coordinate.
    x_mode: Int
        x direction mode.
        If negative, the result wire will have width less than or equal to the given width.
        If positive, the result wire will have width greater than or equal to the given width.
    y_mode: Int
        y direction mode.
        If negative, the result wire will have width less than or equal to the given width.
        If positive, the result wire will have width greater than or equal to the given width.
    half_track: Bool
        True to get half track.
    tr_list: List[Union[Int, float]]
        Wire array track list.
    sp_list: List[Union[HalfInt]]
        Wire array track separation list.
    max_mode: Bool
        True to draw wire with max width from coordinates and tr_list
    track_mode:
        #######
    min_len_mode: Int
        Mininum length mode. See connect_to_tracks for details.
    mode: Int
        draw stack mode.
    Returns
    -------
    wire_arr_list: List[WireArray]
        stack wire list.
    """
    bot_layid = wire.layer_id
    # get wire layer
    if top_layid > bot_layid:
        layer_flip = False
    else:
        layer_flip = True

    if tr_list is not None:
        if len(tr_list) != top_layid - bot_layid:
            raise ValueError('If given tr_list, its length should same as layers(top_layid-bot_layid)')

    # get coordinate
    if x0 is None:
        x0 = wire.bound_box.xl
    if y0 is None:
        y0 = wire.bound_box.yl
    if x1 is None:
        x1 = wire.bound_box.xh
    if y1 is None:
        y1 = wire.bound_box.yh

    # check coordinates
    if x1 <= x0 or y1 <= y0:
        raise ValueError("If given coordinates,"
                         "we need left coord smaller than right coordinate\n"
                         "and bottom coordinate smaller than top coordinate")
    if bot_layid == top_layid:
        raise ValueError("Need top_layer != wire layer. It can be larger or smaller than it.")

    # draw stack wires
    wire_arr_list = [wire]
    if not layer_flip:
        swp_list = list(range(bot_layid, top_layid))
    else:
        swp_list = list(range(top_layid, bot_layid))[::-1]

    for i in swp_list:
        if self.grid.get_direction(i + 1) == Orient2D.y:
            if mode == 0:
                tr, tr_w = self.grid.interval_to_track(i + 1, (x0, x1))
                if tr_w is None:
                    tr_w = 1
                # could specify tr from outside and choose the larger one
                if tr_list is not None:
                    if tr_list[i - bot_layid] is not None:
                        if max_mode:
                            tr_w = max(tr_w, tr_list[i - bot_layid])
                        else:
                            tr_w = tr_list[i - bot_layid]

                tr_tid = TrackID(i + 1, tr, width=tr_w)
                wire_n = self.connect_to_tracks(wire_arr_list[-1], tr_tid, track_lower=y0, track_upper=y1,
                                                min_len_mode=min_len_mode)
            elif mode == 1:
                # get wire width and space
                if tr_list is not None:
                    w_ntr = tr_list[i - bot_layid]
                else:
                    w_ntr = wire.bound_box.w
                if sp_list is not None:
                    sp_ntr = sp_list[i - bot_layid]
                else:
                    sp_ntr = self.grid.get_sep_tracks(i + 1, ntr1=w_ntr, ntr2=w_ntr)
                if sp_margin_list is not None:
                    sp_margin_ntr = sp_margin_list[i - bot_layid]
                else:
                    sp_margin_ntr = sp_ntr // 2
                tid_lower = self.grid.coord_to_track(i + 1, x0, mode=RoundMode.GREATER_EQ)
                tid_upper = self.grid.coord_to_track(i + 1, x1, mode=RoundMode.LESS_EQ)
                if tid_lower == tid_upper:
                    tr_idx = [tid_upper]
                else:
                    tr_idx = self.get_available_tracks(i + 1, tid_lower, tid_upper, y0 - 170, y1 + 170, w_ntr, sep=sp_ntr,
                                                       sep_margin=sp_margin_ntr if sep_margin else None)
                    if tid_upper - tid_lower < w_ntr and len(tr_idx) > 0:
                        tr_idx = [(tid_upper + tid_lower) / 2]
                wire_n = []
                for idx in tr_idx:
                    tr_tid = TrackID(i + 1, idx, width=w_ntr)
                    wire_n.append(self.connect_to_tracks(wire_arr_list[-1], tr_tid, min_len_mode=min_len_mode))
            else:
                raise ValueError("For now, only support two modes.")
        else:
            if mode == 0:
                tr, tr_w = self.grid.interval_to_track(i + 1, (y0, y1))
                if tr_w is None:
                    tr_w = 1
                # could specify tr from outside and choose the larger one
                if tr_list is not None:
                    if tr_list[i - bot_layid] is not None:
                        if max_mode:
                            tr_w = max(tr_w, tr_list[i - bot_layid])
                        else:
                            tr_w = tr_list[i - bot_layid]
                tr_tid = TrackID(i + 1, tr, width=tr_w)
                wire_n = self.connect_to_tracks(wire_arr_list[-1], tr_tid, track_lower=x0, track_upper=x1,
                                                min_len_mode=min_len_mode)
            elif mode == 1:
                # get wire width and space
                if tr_list is not None:
                    w_ntr = tr_list[i - bot_layid]
                else:
                    w_ntr = wire.bound_box.w
                if sp_list is not None:
                    sp_ntr = sp_list[i - bot_layid]
                else:
                    sp_ntr = self.grid.get_sep_tracks(i + 1, ntr1=w_ntr, ntr2=w_ntr)
                if sp_margin_list is not None:
                    sp_margin_ntr = sp_margin_list[i - bot_layid]
                else:
                    sp_margin_ntr = sp_ntr // 2
                tid_lower = self.grid.coord_to_track(i + 1, y0, mode=RoundMode.GREATER_EQ)
                tid_upper = self.grid.coord_to_track(i + 1, y1, mode=RoundMode.LESS_EQ)
                if tid_upper == tid_lower:
                    tr_idx = [tid_lower]
                else:
                    tr_idx = self.get_available_tracks(i + 1, tid_lower, tid_upper, x0 - 150, x1 + 150, w_ntr, sep=sp_ntr,
                                                       sep_margin=sp_margin_ntr if sep_margin else None)
                    if tid_upper - tid_lower < w_ntr and len(tr_idx) > 0:
                        tr_idx = [(tid_upper + tid_lower) / 2]


                wire_n = []
                for idx in tr_idx:
                    tr_tid = TrackID(i + 1, idx, width=w_ntr)
                    wire_n.append(self.connect_to_tracks(wire_arr_list[-1], tr_tid, min_len_mode=min_len_mode, track_lower=x0, track_upper=x1))
            else:
                raise ValueError("For now, only support two modes.")

        wire_arr_list.append(wire_n)

    return wire_arr_list


    # def do_power_fill_poly(self,  # type: TemplateBase
    #                        layer_id,  # type: int
    #                        space,  # type: Union[float, int]
    #                        space_le,  # type: Union[float, int]
    #                        vdd_warrs=None,  # type: Optional[Union[WireArray, List[WireArray]]]
    #                        vss_warrs=None,  # type: Optional[Union[WireArray, List[WireArray]]]
    #                        bound_box_list=None,  # type: Optional[BBox]
    #                        exclude_box_list=[],
    #                        fill_width=1,  # type: int
    #                        fill_space=0,  # type: int
    #                        tr_offset=0,  # type: Union[float, int]
    #                        min_len=0,  # type: Union[float, int]
    #                        flip=False,  # type: bool
    #                        unit_mode=False,  # type: bool
    #                        vss_only=False,
    #                        vdd_only=False,
    #                        ):
    #     # type: (...) -> Tuple[List[WireArray], List[WireArray]]
    #     """Draw power fill on the given layer."""
    #     res = self.grid.resolution
    #     if not unit_mode:
    #         space = int(round(space / res))
    #         space_le = int(round(space_le / res))
    #         x_margin = int(round(x_margin / res))
    #         y_margin = int(round(y_margin / res))
    #         tr_offset = int(round(tr_offset / res))
    #         min_len = int(round(min_len / res))
    #
    #     min_len_process = self.grid.get_min_length(layer_id, fill_width, unit_mode=True)
    #     min_len_global = max(min_len, min_len_process)
    #
    #     if bound_box_list is None:
    #         bound_box_list = [self.bound_box]
    #
    #     cut_out_bound_box_list = self.bbox_list_subtract(bound_box_list, exclude_box_list)
    #
    #     # get absolute 0 half track index
    #     tr_off = self.grid.coord_to_track(layer_id, tr_offset, unit_mode=True)
    #     htr0 = int(tr_off * 2) + 1 + fill_width + fill_space
    #     htr_pitch = 2 * (fill_width + fill_space)
    #     is_horizontal = (self.grid.get_direction(layer_id) == 'x')
    #
    #     top_vdd, top_vss = [], []
    #
    #     for bound_box in cut_out_bound_box_list:
    #
    #         if is_horizontal:
    #             cl, cu = bound_box.bottom_unit, bound_box.top_unit
    #             lower, upper = bound_box.left_unit, bound_box.right_unit
    #             if bound_box.width_unit < min_len_global:
    #                 min_len = min_len_process
    #                 print("warning, bbbox is less wide than min_len, overriding min_len")
    #             else:
    #                 min_len = min_len_global
    #         else:
    #             cl, cu = bound_box.left_unit, bound_box.right_unit
    #             lower, upper = bound_box.bottom_unit, bound_box.top_unit
    #             if bound_box.height_unit < min_len_global:
    #                 min_len = min_len_process
    #                 print("warning, bbbox is less tall than min_len, overriding min_len")
    #             else:
    #                 min_len = min_len_global
    #
    #         # get max and min track
    #         tr_bot = self.grid.find_next_track(layer_id, cl, tr_width=fill_width, half_track=True,
    #                                            mode=1, unit_mode=True)
    #         tr_top = self.grid.find_next_track(layer_id, cu, tr_width=fill_width, half_track=True,
    #                                            mode=-1, unit_mode=True)
    #
    #         # go thru each track and get the open intervals
    #         n0 = - (-(int(tr_bot * 2) + 1 - htr0) // htr_pitch)
    #         n1 = (int(tr_top * 2) + 1 - htr0) // htr_pitch
    #
    #         for ncur in range(n0, n1 + 1):
    #             tr_idx = (htr0 + ncur * htr_pitch - 1) / 2
    #             tid = TrackID(layer_id, tr_idx, width=fill_width)
    #
    #             cur_list = top_vss if (ncur % 2 == 0) != flip else top_vdd
    #
    #             # vdd_only or vdd_only
    #             cur_list = top_vss if vss_only else cur_list
    #             cur_list = top_vdd if vdd_only else cur_list
    #             if vss_only and vdd_only:
    #                 raise ValueError("only one of 'vss_only' and 'vdd_only' could be True.")
    #
    #             for tl, tu in self.open_interval_iter(tid, lower, upper, sp=space, sp_le=space_le,
    #                                                   min_len=min_len):
    #                 cur_list.append(WireArray(tid, tl, tu, res=res, unit_mode=True))
    #
    #     # draw the grid rects for each interval
    #     for warr in chain(top_vdd, top_vss):
    #         for lay, box_arr in warr.wire_arr_iter(self.grid):
    #             self.add_rect(lay, box_arr)
    #
    #     # drop vias
    #     if vdd_warrs:
    #         self.draw_vias_on_intersections(vdd_warrs, top_vdd)
    #     if vss_warrs:
    #         self.draw_vias_on_intersections(vss_warrs, top_vss)
    #
    #     return top_vdd, top_vss
    #
    # def bbox_list_subtract(self, bbox_a_list, bbox_b_list):
    #     bbox_results = bbox_a_list
    #
    #     for bbox_b in bbox_b_list:
    #         bbox_results_new = []
    #
    #         for bbox_a in bbox_results:
    #             #    // h +-+-+-+
    #             #    // . |6|7|8|
    #             #    // g +-+-+-+
    #             #    // . |3|4|5|
    #             #    // f +-+-+-+
    #             #    // . |0|1|2|
    #             #    // e +-+-+-+
    #             #    // . a b c d
    #
    #             a = min(bbox_a.left_unit, bbox_b.left_unit)
    #             if (bbox_a.right_unit > bbox_b.left_unit) and (bbox_b.right_unit > bbox_a.left_unit):
    #                 b = max(bbox_a.left_unit, bbox_b.left_unit)
    #                 c = min(bbox_a.right_unit, bbox_b.right_unit)
    #                 x_overlap = True
    #             else:
    #                 c = max(bbox_a.left_unit, bbox_b.left_unit)
    #                 b = min(bbox_a.right_unit, bbox_b.right_unit)
    #                 x_overlap = False
    #             d = max(bbox_a.right_unit, bbox_b.right_unit)
    #
    #             e = min(bbox_a.bottom_unit, bbox_b.bottom_unit)
    #             if (bbox_a.top_unit > bbox_b.bottom_unit) and (bbox_b.top_unit > bbox_a.bottom_unit):
    #                 f = max(bbox_a.bottom_unit, bbox_b.bottom_unit)
    #                 g = min(bbox_a.top_unit, bbox_b.top_unit)
    #                 y_overlap = True
    #             else:
    #                 g = max(bbox_a.bottom_unit, bbox_b.bottom_unit)
    #                 f = min(bbox_a.top_unit, bbox_b.top_unit)
    #                 y_overlap = False
    #             h = max(bbox_a.top_unit, bbox_b.top_unit)
    #
    #             if y_overlap and x_overlap:
    #
    #                 colA_trivial = (a == b)
    #                 colB_trivial = (b == c)
    #                 colC_trivial = (c == d)
    #
    #                 rowA_trivial = (e == f)
    #                 rowB_trivial = (f == g)
    #                 rowC_trivial = (g == h)
    #
    #                 BRect0 = BBox(left=a, bottom=e, right=b, top=f, resolution=self.grid.resolution,
    #                               unit_mode=True) if not (rowA_trivial or colA_trivial) else None
    #                 BRect1 = BBox(left=b, bottom=e, right=c, top=f, resolution=self.grid.resolution,
    #                               unit_mode=True) if not (rowA_trivial or colB_trivial) else None
    #                 BRect2 = BBox(left=c, bottom=e, right=d, top=f, resolution=self.grid.resolution,
    #                               unit_mode=True) if not (rowA_trivial or colC_trivial) else None
    #                 BRect3 = BBox(left=a, bottom=f, right=b, top=g, resolution=self.grid.resolution,
    #                               unit_mode=True) if not (rowB_trivial or colA_trivial) else None
    #                 BRect4 = BBox(left=b, bottom=f, right=c, top=g, resolution=self.grid.resolution,
    #                               unit_mode=True) if not (rowB_trivial or colB_trivial) else None
    #                 BRect5 = BBox(left=c, bottom=f, right=d, top=g, resolution=self.grid.resolution,
    #                               unit_mode=True) if not (rowB_trivial or colC_trivial) else None
    #                 BRect6 = BBox(left=a, bottom=g, right=b, top=h, resolution=self.grid.resolution,
    #                               unit_mode=True) if not (rowC_trivial or colA_trivial) else None
    #                 BRect7 = BBox(left=b, bottom=g, right=c, top=h, resolution=self.grid.resolution,
    #                               unit_mode=True) if not (rowC_trivial or colB_trivial) else None
    #                 BRect8 = BBox(left=c, bottom=g, right=d, top=h, resolution=self.grid.resolution,
    #                               unit_mode=True) if not (rowC_trivial or colC_trivial) else None
    #
    #                 if not (BRect0 is None):
    #                     if (bbox_a.left_unit < bbox_b.left_unit) and (bbox_a.bottom_unit < bbox_b.bottom_unit):
    #                         bbox_results_new.append(BRect0)
    #
    #                 if not (BRect1 is None):
    #                     if (bbox_a.bottom_unit < bbox_b.bottom_unit) and x_overlap:
    #                         bbox_results_new.append(BRect1)
    #
    #                 if not (BRect2 is None):
    #                     if (bbox_a.right_unit > bbox_b.right_unit) and (bbox_a.bottom_unit < bbox_b.bottom_unit):
    #                         bbox_results_new.append(BRect2)
    #
    #                 if not (BRect3 is None):
    #                     if (bbox_a.left_unit < bbox_b.left_unit) and y_overlap:
    #                         bbox_results_new.append(BRect3)
    #
    #                 if not (BRect4 is None):
    #                     if False:
    #                         bbox_results_new.append(BRect4)
    #
    #                 if not (BRect5 is None):
    #                     if (bbox_a.right_unit > bbox_b.right_unit) and y_overlap:
    #                         bbox_results_new.append(BRect5)
    #
    #                 if not (BRect6 is None):
    #                     if (bbox_a.left_unit < bbox_b.left_unit) and (bbox_a.top_unit > bbox_b.top_unit):
    #                         bbox_results_new.append(BRect6)
    #
    #                 if not (BRect7 is None):
    #                     if (bbox_a.top_unit > bbox_b.top_unit) and x_overlap:
    #                         bbox_results_new.append(BRect7)
    #
    #                 if not (BRect8 is None):
    #                     if (bbox_a.right_unit > bbox_b.right_unit) and (bbox_a.top_unit > bbox_b.top_unit):
    #                         bbox_results_new.append(BRect8)
    #             else:
    #                 bbox_results_new.append(bbox_a)
    #
    #         bbox_results = bbox_results_new
    #
    #     for bbox in bbox_results:
    #         if (bbox.left_unit >= bbox.right_unit) or (bbox.bottom_unit >= bbox.top_unit):
    #             raise Exception('bbox subtraction screwed up, max dim must be greater than min dim')
    #
    #     return bbox_results
    #
    # def draw_bump_rv(self, pad_diameter, pad_center, bump_layer, wing_size, rv_land_width, rv_land_space,
    #                  remove_wings=[0, 0, 0, 0], bump_net=None, show_pins=False):
    #
    #     pad_radius = int(np.floor(pad_diameter / 2))
    #     pad_half_side = int(np.floor(0.5 * pad_diameter / (1 + np.sqrt(2))))
    #     pad_layer_name = self.grid.get_layer_name(bump_layer, 0)
    #     pad_center_x = pad_center[0]
    #     pad_center_y = pad_center[1]
    #
    #     draw_left_wing = not (bool(remove_wings[0]))
    #     draw_bot_wing = not (bool(remove_wings[1]))
    #     draw_right_wing = not (bool(remove_wings[2]))
    #     draw_top_wing = not (bool(remove_wings[3]))
    #
    #     wire_drops = []
    #     wire_drop_boxes = []
    #
    #     bot_layer = self.grid.get_layer_name(bump_layer - 1, 0)
    #     top_layer = self.grid.get_layer_name(bump_layer, 0)
    #
    #     rv_land_pitch = rv_land_width + rv_land_space
    #
    #     if draw_top_wing:
    #         top_pad_wing = self.add_rect(pad_layer_name, BBox(
    #             left=pad_center_x - pad_half_side, bottom=pad_center_y + pad_radius,
    #             right=pad_center_x + pad_half_side, top=pad_center_y + pad_radius + wing_size,
    #             resolution=self.grid.resolution, unit_mode=True
    #         ))
    #
    #         wire_drop_boxes.append(top_pad_wing.bbox)
    #         top_wing_lower_index = self.grid.coord_to_nearest_track(
    #             layer_id=bump_layer - 1,
    #             coord=top_pad_wing.bbox.left_unit,
    #             mode=1,
    #             half_track=False,
    #             unit_mode=True,
    #         )
    #         top_wing_upper_index = self.grid.coord_to_nearest_track(
    #             layer_id=bump_layer - 1,
    #             coord=top_pad_wing.bbox.right_unit,
    #             mode=-1,
    #             half_track=False,
    #             unit_mode=True,
    #         )
    #
    #         top_wing_span = np.floor(
    #             (top_wing_upper_index - top_wing_lower_index - rv_land_width) / rv_land_pitch) * rv_land_pitch + 1
    #         top_wing_center = (top_wing_upper_index + top_wing_lower_index) / 2
    #         top_wing_lower_range = int(np.ceil(top_wing_center - top_wing_span / 2))
    #         top_wing_upper_range = int(np.floor(top_wing_center + top_wing_span / 2)) + 1
    #
    #         for track_index in range(top_wing_lower_range, top_wing_upper_range, rv_land_pitch):
    #             temp_warr = self.add_wires(
    #                 layer_id=bump_layer - 1,
    #                 track_idx=track_index,
    #                 lower=top_pad_wing.bbox.bottom_unit,
    #                 upper=top_pad_wing.bbox.top_unit,
    #                 width=rv_land_width,
    #                 unit_mode=True,
    #             )
    #             self.add_via(bbox=temp_warr.get_bbox_array(self.grid).get_bbox(0), extend=False, bot_layer=bot_layer,
    #                          top_layer=top_layer, bot_dir='y')
    #             wire_drops.append(temp_warr)
    #
    #     if draw_bot_wing:
    #         bottom_pad_wing = self.add_rect(pad_layer_name, BBox(
    #             left=pad_center_x - pad_half_side, bottom=pad_center_y - pad_radius - wing_size,
    #             right=pad_center_x + pad_half_side, top=pad_center_y - pad_radius,
    #             resolution=self.grid.resolution, unit_mode=True
    #         ))
    #
    #         wire_drop_boxes.append(bottom_pad_wing.bbox)
    #         bottom_wing_lower_index = self.grid.coord_to_nearest_track(
    #             layer_id=bump_layer - 1,
    #             coord=bottom_pad_wing.bbox.left_unit,
    #             mode=1,
    #             half_track=False,
    #             unit_mode=True,
    #         )
    #         bottom_wing_upper_index = self.grid.coord_to_nearest_track(
    #             layer_id=bump_layer - 1,
    #             coord=bottom_pad_wing.bbox.right_unit,
    #             mode=-1,
    #             half_track=False,
    #             unit_mode=True,
    #         )
    #
    #         bottom_wing_span = np.floor(
    #             (bottom_wing_upper_index - bottom_wing_lower_index - rv_land_width) / rv_land_pitch) * rv_land_pitch + 1
    #         bottom_wing_center = (bottom_wing_upper_index + bottom_wing_lower_index) / 2
    #         bottom_wing_lower_range = int(np.ceil(bottom_wing_center - bottom_wing_span / 2))
    #         bottom_wing_upper_range = int(np.floor(bottom_wing_center + bottom_wing_span / 2)) + 1
    #
    #         for track_index in range(bottom_wing_lower_range, bottom_wing_upper_range, rv_land_pitch):
    #             temp_warr = self.add_wires(
    #                 layer_id=bump_layer - 1,
    #                 track_idx=track_index,
    #                 lower=bottom_pad_wing.bbox.bottom_unit,
    #                 upper=bottom_pad_wing.bbox.top_unit,
    #                 width=rv_land_width,
    #                 unit_mode=True,
    #             )
    #             self.add_via(bbox=temp_warr.get_bbox_array(self.grid).get_bbox(0), extend=False, bot_layer=bot_layer,
    #                          top_layer=top_layer, bot_dir='y')
    #             wire_drops.append(temp_warr)
    #
    #     if draw_left_wing:
    #         left_pad_wing = self.add_rect(pad_layer_name, BBox(
    #             left=pad_center_x - pad_radius - wing_size, bottom=pad_center_y - pad_half_side,
    #             right=pad_center_x - pad_radius, top=pad_center_y + pad_half_side,
    #             resolution=self.grid.resolution, unit_mode=True
    #         ))
    #
    #         wire_drop_boxes.append(left_pad_wing.bbox)
    #         left_wing_lower_index = self.grid.coord_to_nearest_track(
    #             layer_id=bump_layer - 1,
    #             coord=left_pad_wing.bbox.left_unit,
    #             mode=1,
    #             half_track=False,
    #             unit_mode=True,
    #         )
    #         left_wing_upper_index = self.grid.coord_to_nearest_track(
    #             layer_id=bump_layer - 1,
    #             coord=left_pad_wing.bbox.right_unit,
    #             mode=-1,
    #             half_track=False,
    #             unit_mode=True,
    #         )
    #
    #         left_wing_lower_range = left_wing_lower_index
    #         left_wing_upper_range = left_wing_upper_index + 2 - rv_land_width
    #         for left_index in range(left_wing_lower_range, left_wing_upper_range, rv_land_pitch):
    #             track_index = left_index + np.ceil(rv_land_width / 2 - .5)
    #             temp_warr = self.add_wires(
    #                 layer_id=bump_layer - 1,
    #                 track_idx=track_index,
    #                 lower=left_pad_wing.bbox.bottom_unit,
    #                 upper=left_pad_wing.bbox.top_unit,
    #                 width=rv_land_width,
    #                 unit_mode=True,
    #             )
    #             self.add_via(bbox=temp_warr.get_bbox_array(self.grid).get_bbox(0), extend=False, bot_layer=bot_layer,
    #                          top_layer=top_layer, bot_dir='y')
    #             wire_drops.append(temp_warr)
    #
    #     if draw_right_wing:
    #         right_pad_wing = self.add_rect(pad_layer_name, BBox(
    #             left=pad_center_x + pad_radius, bottom=pad_center_y - pad_half_side,
    #             right=pad_center_x + pad_radius + wing_size, top=pad_center_y + pad_half_side,
    #             resolution=self.grid.resolution, unit_mode=True
    #         ))
    #
    #         wire_drop_boxes.append(right_pad_wing.bbox)
    #         right_wing_lower_index = self.grid.coord_to_nearest_track(
    #             layer_id=bump_layer - 1,
    #             coord=right_pad_wing.bbox.left_unit,
    #             mode=1,
    #             half_track=False,
    #             unit_mode=True,
    #         )
    #         right_wing_upper_index = self.grid.coord_to_nearest_track(
    #             layer_id=bump_layer - 1,
    #             coord=right_pad_wing.bbox.right_unit,
    #             mode=-1,
    #             half_track=False,
    #             unit_mode=True,
    #         )
    #
    #         right_wing_lower_range = right_wing_upper_index
    #         right_wing_upper_range = right_wing_lower_index - 2 + rv_land_width
    #         for right_index in range(right_wing_lower_range, right_wing_upper_range, -1 * rv_land_pitch):
    #             track_index = right_index - np.floor(rv_land_width / 2)
    #             temp_warr = self.add_wires(
    #                 layer_id=bump_layer - 1,
    #                 track_idx=track_index,
    #                 lower=right_pad_wing.bbox.bottom_unit,
    #                 upper=right_pad_wing.bbox.top_unit,
    #                 width=rv_land_width,
    #                 unit_mode=True,
    #             )
    #             self.add_via(bbox=temp_warr.get_bbox_array(self.grid).get_bbox(0), extend=False, bot_layer=bot_layer,
    #                          top_layer=top_layer, bot_dir='y')
    #             wire_drops.append(temp_warr)
    #
    #     return wire_drops, wire_drop_boxes
    #
    # def floorplan_bbox_transform(self, base_box_list, position, orient):
    #     new_box_list = []
    #     for base_box in base_box_list:
    #         if orient == 'R0':
    #             rotated_box = base_box
    #         elif orient == 'MY':
    #             rotated_box = [-base_box[2], base_box[1], -base_box[0], base_box[3]]
    #         elif orient == 'MX':
    #             rotated_box = [base_box[0], -base_box[3], base_box[2], -base_box[1]]
    #         elif orient == 'R180':
    #             rotated_box = [-base_box[2], -base_box[3], -base_box[0], -base_box[1]]
    #         else:
    #             raise ("bad rotation value")
    #
    #         new_box = [rotated_box[0] + position[0], rotated_box[1] + position[1], rotated_box[2] + position[0],
    #                    rotated_box[3] + position[1]]
    #         new_box_list.append(new_box)
    #     return new_box_list
    #
    # def create_global_grid(self, tr_manager, moved_macro_grid_poly_phrb, phrb_width, phrb_height, top_layer,
    #                        glob_min_grid_layer, glob_max_grid_layer,
    #                        macro_instance_list, bump_grid_vdd_list, bump_grid_vss_list, show_pins=True,
    #                        pin_missed_rails=True):
    #
    #     # Create global Grid
    #     top_global_vdd_warrs = []
    #     top_global_vss_warrs = []
    #     power_island_box_list = []
    #
    #     for power_island_points in moved_macro_grid_poly_phrb:
    #         power_island_box = BBox(
    #             left=power_island_points[0] * phrb_width, bottom=power_island_points[1] * phrb_height,
    #             right=power_island_points[2] * phrb_width, top=power_island_points[3] * phrb_height,
    #             resolution=self.grid.resolution, unit_mode=True
    #         )
    #         power_island_box_list.append(power_island_box)
    #         self.add_rect(('ref', 'drawing'), power_island_box)
    #
    #     [blk_w, blk_h] = self.grid.get_block_size(top_layer, unit_mode=True)
    #
    #     warr_vdd = []
    #     warr_vss = []
    #     for power_layer in range(glob_min_grid_layer, glob_max_grid_layer + 1):
    #
    #         power_grid_width_tracks = tr_manager.get_width(layer_id=power_layer, track_type='supply')
    #         power_grid_supply_space_tracks = tr_manager.get_space(layer_id=power_layer, type_tuple=('supply', 'supply'))
    #         power_grid_sig_space_tracks = tr_manager.get_space(layer_id=power_layer, type_tuple=('supply', ''))
    #
    #         blk_snap = (power_grid_width_tracks + power_grid_supply_space_tracks) * self.grid.get_track_pitch(
    #             power_layer, unit_mode=True)
    #
    #         # exclude_boxes = []
    #         exclude_boxes_snapped = []
    #         # exclude_boxes = master_serdes_rx.grid_exclude_boxes_per_layer.get(power_layer,[])
    #
    #         for macro_instance in macro_instance_list:
    #             macro_master = macro_instance.master
    #             exclude_boxes = macro_master.grid_exclude_boxes_per_layer.get(power_layer, []) if hasattr(macro_master,
    #                                                                                                       'grid_exclude_boxes_per_layer') else []
    #
    #             for old_box in exclude_boxes:
    #                 relocate_box = old_box[0].transform(loc=macro_instance.location_unit,
    #                                                     orient=macro_instance.orientation, unit_mode=True)
    #
    #                 self.add_rect(('DMEXCL', 'dummy' + str(power_layer)), relocate_box)
    #
    #                 blk_snap_left = -1 * blk_snap * old_box[1][0] if not (old_box[1][0] == 0) else 1
    #                 blk_snap_bottom = -1 * blk_snap * old_box[1][1] if not (old_box[1][1] == 0) else 1
    #                 blk_snap_right = -1 * blk_snap * old_box[1][2] if not (old_box[1][2] == 0) else 1
    #                 blk_snap_top = -1 * blk_snap * old_box[1][3] if not (old_box[1][3] == 0) else 1
    #
    #                 exclude_boxes_snapped.append(BBox(
    #                     left=relocate_box.left_unit // blk_snap_left * blk_snap_left,
    #                     bottom=relocate_box.bottom_unit // blk_snap_bottom * blk_snap_bottom,
    #                     right=relocate_box.right_unit // blk_snap_right * blk_snap_right,
    #                     top=relocate_box.top_unit // blk_snap_top * blk_snap_top,
    #                     resolution=self.grid.resolution, unit_mode=True,
    #                 ))
    #
    #         print('adding grind on %d' % power_layer)
    #         # pdb.set_trace()
    #
    #         # if power_layer == 8:
    #         #    exclude_boxes_snapped_test = []
    #         #    exclude_boxes_snapped_test.append(BBox(
    #         #                        left  = 211680,  bottom = 705600,
    #         #                        right = 330120,  top    = 813960,
    #         #                        resolution=self.grid.resolution, unit_mode=True,
    #         #    ))
    #         #    exclude_boxes_snapped_test.append(BBox(
    #         #                        left  = 483840,  bottom = 695520,
    #         #                        right = 624960,  top    = 824040,
    #         #                        resolution=self.grid.resolution, unit_mode=True,
    #         #    ))
    #         #    exclude_boxes_snapped_test.append(BBox(
    #         #                        left  = 211680,  bottom = 740880,
    #         #                        right = 388080,  top    = 778680,
    #         #                        resolution=self.grid.resolution, unit_mode=True,
    #         #    ))
    #         #    exclude_boxes_snapped_test.append(BBox(
    #         #                        left  = 388080,  bottom = 720720,
    #         #                        right = 624960,  top    = 796320,
    #         #                        resolution=self.grid.resolution, unit_mode=True,
    #         #    ))
    #
    #         # elif power_layer == 9:
    #
    #         #    exclude_boxes_snapped_test = []
    #         #    exclude_boxes_snapped_test.append(BBox(
    #         #                        left  = 388080,  bottom = 727215,
    #         #                        right = 624960,  top    = 791505,
    #         #                        resolution=self.grid.resolution, unit_mode=True,
    #         #    ))
    #         #    exclude_boxes_snapped_test.append(BBox(
    #         #                        left  = 211680,  bottom = 740880,
    #         #                        right = 374865,  top    = 777840,
    #         #                        resolution=self.grid.resolution, unit_mode=True,
    #         #    ))
    #         #    exclude_boxes_snapped_test.append(BBox(
    #         #                        left  = 211680,  bottom = 746955,
    #         #                        right = 388080,  top    = 770925,
    #         #                        resolution=self.grid.resolution, unit_mode=True,
    #         #    ))
    #         #    exclude_boxes_snapped_test.append(BBox(
    #         #                        left  = 211680,  bottom = 705600,
    #         #                        right = 330120,  top    = 813960,
    #         #                        resolution=self.grid.resolution, unit_mode=True,
    #         #    ))
    #         #    exclude_boxes_snapped_test.append(BBox(
    #         #                        left  = 483840,  bottom = 695520,
    #         #                        right = 624960,  top    = 824040,
    #         #                        resolution=self.grid.resolution, unit_mode=True,
    #         #    ))
    #         # exclude_boxes_snapped = exclude_boxes_snapped_test
    #
    #         if power_layer == top_layer:
    #             min_len = 1.5 * (tr_manager.get_width(layer_id=power_layer, track_type='supply') + tr_manager.get_space(
    #                 layer_id=power_layer, type_tuple=('supply', 'supply'))) * self.grid.get_track_pitch(power_layer,
    #                                                                                                     unit_mode=True)
    #             print('wiring gird on top layer')
    #         else:
    #             min_len = 1.5 * (
    #                         tr_manager.get_width(layer_id=power_layer + 1, track_type='supply') + tr_manager.get_space(
    #                     layer_id=power_layer + 1, type_tuple=('supply', 'supply'))) * self.grid.get_track_pitch(
    #                 power_layer + 1, unit_mode=True)
    #
    #         warr_vdd, warr_vss = self.do_power_fill_poly(
    #             layer_id=power_layer,
    #             space=power_grid_sig_space_tracks * self.grid.get_track_pitch(power_layer, unit_mode=True),
    #             space_le=3 * self.grid.get_line_end_space(power_layer, power_grid_width_tracks, unit_mode=True),
    #             vdd_warrs=warr_vdd,
    #             vss_warrs=warr_vss,
    #             bound_box_list=power_island_box_list,
    #             exclude_box_list=exclude_boxes_snapped,
    #             fill_width=power_grid_width_tracks,
    #             fill_space=power_grid_supply_space_tracks,
    #             unit_mode=True,
    #             min_len=min_len
    #         )
    #
    #         if power_layer == glob_max_grid_layer - 1:
    #             if bump_grid_vdd_list:
    #                 self.draw_vias_on_intersections(warr_vdd, bump_grid_vdd_list)
    #
    #             if bump_grid_vss_list:
    #                 self.draw_vias_on_intersections(warr_vss, bump_grid_vss_list)
    #
    #         if True:
    #             sub_block_upper_vdd_warrs = []
    #             sub_block_upper_vss_warrs = []
    #             for macro_instance in macro_instance_list:
    #                 sub_block_upper_vdd_warrs.extend(macro_instance.get_all_port_pins('VDD', layer=power_layer - 1))
    #                 sub_block_upper_vss_warrs.extend(macro_instance.get_all_port_pins('VSS', layer=power_layer - 1))
    #
    #             if sub_block_upper_vdd_warrs:
    #                 sub_block_vdd_conns_made = self.draw_vias_on_intersections(sub_block_upper_vdd_warrs, warr_vdd)
    #
    #                 print('vdd')
    #                 for y, x in zip(sub_block_vdd_conns_made, sub_block_upper_vdd_warrs):
    #                     if not (y):
    #                         print(x.get_bbox_array(self.grid))
    #                         if pin_missed_rails:
    #                             self.add_pin('VDD', x, label='VDD:')
    #                         else:
    #                             for lay, box_arr in x.wire_arr_iter(self.grid):
    #                                 self.add_rect(lay, box_arr)
    #
    #             if sub_block_upper_vss_warrs:
    #                 sub_block_vss_conns_made = self.draw_vias_on_intersections(sub_block_upper_vss_warrs, warr_vss)
    #
    #                 print('vss')
    #                 for y, x in zip(sub_block_vss_conns_made, sub_block_upper_vss_warrs):
    #                     if not (y):
    #                         print(x.get_bbox_array(self.grid))
    #                         if pin_missed_rails:
    #                             self.add_pin('VSS', x, label='VSS:')
    #                         else:
    #                             for lay, box_arr in x.wire_arr_iter(self.grid):
    #                                 self.add_rect(lay, box_arr)
    #
    #         top_global_vdd_warrs.extend(warr_vdd)
    #         top_global_vss_warrs.extend(warr_vss)
    #
    #     if pin_missed_rails:
    #         self.add_pin('VDD', top_global_vdd_warrs, label='VDD:')
    #         self.add_pin('VSS', top_global_vss_warrs, label='VSS:')
    #     else:
    #         self.add_pin('VDD', top_global_vdd_warrs)
    #         self.add_pin('VSS', top_global_vss_warrs)
    #
    # def add_bump_hookup(self, top_layer, pad_diameter, wing_size, rv_land_width, rv_land_space, bump_x, bump_y,
    #                     bump_net, remove_wings, show_pins):
    #
    #     pad_radius = int(np.floor(pad_diameter / 2))
    #     pad_half_side = int(np.floor(0.5 * pad_diameter / (1 + np.sqrt(2))))
    #
    #     pad_layer_name = self.grid.get_layer_name(top_layer, 0)
    #
    #     horz_neg_offset = pad_half_side if remove_wings[0] == 1 else pad_radius + wing_size
    #     vert_neg_offset = pad_half_side if remove_wings[1] == 1 else pad_radius + wing_size
    #     horz_pos_offset = pad_half_side if remove_wings[2] == 1 else pad_radius + wing_size
    #     vert_pos_offset = pad_half_side if remove_wings[3] == 1 else pad_radius + wing_size
    #
    #     self.add_rect(pad_layer_name, BBox(
    #         left=bump_x - pad_half_side, bottom=bump_y - vert_neg_offset,
    #         right=bump_x + pad_half_side, top=bump_y + vert_pos_offset,
    #         resolution=self.grid.resolution, unit_mode=True
    #     ))
    #
    #     self.add_rect(pad_layer_name, BBox(
    #         left=bump_x - horz_neg_offset, bottom=bump_y - pad_half_side,
    #         right=bump_x + horz_pos_offset, top=bump_y + pad_half_side,
    #         resolution=self.grid.resolution, unit_mode=True
    #     ))
    #
    #     self.add_rect(pad_layer_name, BBox(
    #         left=bump_x - pad_half_side, bottom=bump_y - pad_half_side,
    #         right=bump_x + pad_half_side, top=bump_y + pad_half_side,
    #         resolution=self.grid.resolution, unit_mode=True
    #     ))
    #
    #     ap_pin = self.add_rect(pad_layer_name, BBox(
    #         left=bump_x - pad_diameter // 32, bottom=bump_y - pad_diameter // 32,
    #         right=bump_x + pad_diameter // 32, top=bump_y + pad_diameter // 32,
    #         resolution=self.grid.resolution, unit_mode=True
    #     ))
    #
    #     # if not((bump_net == 'ANALOG_OUT_N') or (bump_net == 'ANALOG_OUT_P')):
    #     if True:
    #         self.add_pin_primitive(bump_net, pad_layer_name, ap_pin.bbox)
    #
    #     wire_drops, wire_drop_boxes = self.draw_bump_rv(pad_diameter, [bump_x, bump_y], top_layer, wing_size,
    #                                                     rv_land_width, rv_land_space, remove_wings)
    #
    #     return wire_drops, wire_drop_boxes
    #

def draw_multiple_stack_wire(self: TemplateBase, tr_manager, warr_list: List[WireArray], top_layid: int, wire_type: str,
                             sep_type: (str, str), y0=None, y1=None, x0=None, x1=None, stack_idx=-1):

    stack_wire = []
    cur_lay = warr_list[0].layer_id
    layer_range = range(cur_lay + 1, top_layid + 1)
    for warr in warr_list:
        stack = draw_stack_wire(self, warr, top_layid, x0=x0, x1=x1, y0=y0, y1=y1,
                                tr_list=[tr_manager.get_width(lay, wire_type) for lay in layer_range],
                                sp_list=[tr_manager.get_sep(lay, sep_type) for lay in layer_range])
        stack_wire += stack[stack_idx]
    return stack_wire


def extend_matching_wires(self, warrs: List[WireArray], lower: Optional[Union[int, float]]=None,
                          upper: Optional[Union[int, float]]=None)-> List[WireArray]:

    warr_lower = lower if lower!=None else warrs[0].lower
    warr_upper = upper if upper!=None else warrs[0].upper

    lay_id = warrs[0].layer_id
    width = warrs[0].track_id.width
    warr_lower = warr_lower if warrs[0].lower > warr_lower else warrs[0].lower
    warr_upper = warr_upper if warrs[0].upper < warr_upper else warrs[0].upper

    for warr in warrs[1:]:
        # inspect layer id
        assert lay_id == warr.layer_id
        # find max width
        width = warr.track_id.width if width < warr.track_id.width else width
        # find min lower
        warr_lower = warr_lower if warr.lower > warr_lower else warr.lower
        # find max upper
        warr_upper = warr_upper if warr.upper < warr_upper else warr.upper

    extended_warrs = []
    for warr in warrs:
        new_warr = self.add_wires(lay_id, warr.track_id.base_index, warr_lower, warr_upper, width=width)
        extended_warrs.append(new_warr)

    return extended_warrs

def power_fill_helper(self, layer_id: int, tr_manager: TrackManager,
                          vdd_warrs: List[WireArray], vss_warrs: List[WireArray], bbox_overwrite=False,
                          bound_box: Optional[BBox] = None, top_wire: Optional[WireArray] = None,
                          bot_wire: Optional[WireArray] = None, left_wire: Optional[WireArray] = None,
                          right_wire: Optional[WireArray] = None, **kwargs) -> Tuple[List[WireArray], List[WireArray]]:
        """Calculate via extention to do power fill.
        Parameters
        ----------
        layer_id: int
            the layer to draw power lines
        vdd_warrs: List[WireArray]
            vdd wires (layer_id-1) to be connected. [] if no vdd wires
        vss_warrs: List[WireArray]
            vss wires (layer_id-1) to be connected. [] if no vss wires
        bbox_overwrite: bool
            True to overwrite with bound_box
        bound_box: Optional[BBox]
            The region to draw power lines
            (takes the largest margin among bound_box and vdd_warrs/vss_warrs in layer_id direction
             takes the smallest margin among bound_box and vdd_warrs/vss_warrs in layer_id-1 direction)
        top_wire: Optional[WireArray]
            The topmost wire in among vss_warrs/vdd_warrs if not specified, the last element in vdd_warrs or vss_warrs
            would be assigned if vdd_warrs + vss_warrs is even or odd
         bot_wire: Optional[WireArray]
            The bottommost wire in among vss_warrs/vdd_warrs if not specified, the first element in vss_warrs or
            vdd_warrs (if vss_warrs = [])
        left_wire: Optional[WireArray]
            The leftmost wire in among vss_warrs/vdd_warrs if not specified, the first element in vss_warrs or
            vdd_warrs (if vss_warrs = [])
        right_wire: Optional[WireArray]
            The rightmost wire in among vss_warrs/vdd_warrs if not specified, the last element in vdd_warrss or vss_warrs
            would be assigned if vdd_warrs + vss_warrs is even or odd

         Returns
        -------
        (vdd, vss) : Tuple[List[WireArray], List[WireArray]]
            list of created wires on layer_id

        """
        sup_w = tr_manager.get_width(layer_id, 'sup')
        sup_w_lower = tr_manager.get_width(layer_id - 1, 'sup')
        wl, wh = self.grid.get_wire_bounds(layer_id, 0, width=sup_w)
        w_mid = int((wh - wl) / 2)
        via_ext_lower, via_ext_cur = self.grid.get_via_extensions(Direction.LOWER, layer_id - 1, sup_w_lower, sup_w)

        is_horizontal = (self.grid.get_direction(layer_id) == 0)
        num_vss = len(vss_warrs)
        num_vdd = len(vdd_warrs)
        if num_vdd + num_vss == 0:
            raise ValueError("need at least vss_warrs or vdd_wars")

        top_wire, bot_wire, left_wire, right_wire = get_boundary_wires(self, vdd_warrs + vss_warrs)
        # if not top_wire:
        #     top_wire = vss_warrs[-1] if ((num_vss + num_vdd) & 1) and (num_vss > 0) else vdd_warrs[-1]
        # if not bot_wire:
        #     bot_wire = vss_warrs[0] if num_vss > 0 else vdd_warrs[0]
        # if not right_wire:
        #     right_wire = vss_warrs[-1] if ((num_vss + num_vdd) & 1) and (num_vss > 0) else vdd_warrs[-1]
        # if not left_wire:
        #     left_wire = vss_warrs[0] if num_vss > 0 else vdd_warrs[0]
        # layer_id is horizontal
        if is_horizontal:
            dx = via_ext_cur
            dy = - via_ext_lower - w_mid

        # layer_id is vertical
        else:
            dx = - via_ext_lower - w_mid
            dy = via_ext_cur
        xl, xh, yl, yh = left_wire.bound_box.xl, right_wire.bound_box.xh,  bot_wire.bound_box.yl, top_wire.bound_box.yh
        xl = xl if is_horizontal else xl - dx
        xh = xh if is_horizontal else xh + dx
        yl = yl if not is_horizontal else yl - dy
        yh = yh if not is_horizontal else yh + dy

        xl = bound_box.xl if (bound_box and ((bound_box.xl < xl) or bbox_overwrite)) else xl
        xh = bound_box.xh if (bound_box and ((bound_box.xh > xh) or bbox_overwrite)) else xh
        yl = bound_box.yl if (
                    bound_box and ((bound_box.yl < yl) or bbox_overwrite)) else yl
        yh = bound_box.yh if (
                    bound_box and ((bound_box.yh > yh) or bbox_overwrite)) else yh
        # bb = bound_box.expand(dy=dy, dx=dx)
        bb = BBox(xl=xl, yl=yl, xh=xh, yh=yh)
        if num_vss == 0:
            sup_type = 'vdd'
        elif num_vdd == 0:
            sup_type = 'vss'
        else:
            sup_type = 'both'
        vdd, vss = self.do_power_fill(layer_id, tr_manager, vdd_warrs, vss_warrs, bound_box=bb, sup_type=sup_type,
                                      **kwargs)
        return vdd, vss


def get_boundary_wires(self, warrs: List[WireArray]) -> Tuple[WireArray, WireArray, WireArray, WireArray]:
    # return topmost, bottommost, leftmost, and rightmost wire in a list of wires
    # initialize
    wire_top, wire_bot, wire_left, wire_right = [warrs[0]] * 4
    for wire in warrs:
        if wire.bound_box.yh > wire_top.bound_box.yh:
            wire_top = wire
        if wire.bound_box.yl < wire_bot.bound_box.yl:
            wire_bot = wire
        if wire.bound_box.xl < wire_left.bound_box.xl:
            wire_left = wire
        if wire.bound_box.xh > wire_right.bound_box.xh:
            wire_right = wire

    return wire_top, wire_bot, wire_left, wire_right
