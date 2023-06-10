def parse_obj(filename: str) -> list[list[float]]:
    all_lines = list()
    # read all lines
    with open(filename, 'r') as f:
        line = f.readline()
        while len(line) > 0:
            all_lines.append(line)
            line = f.readline()
    all_lines_split = list(map(lambda x: x.split(" "), all_lines))
    vertex_strs = list(filter(lambda x: x[0] == 'v', all_lines_split))
    facet_strs = list(filter(lambda x: x[0] == 'f', all_lines_split))

    facets = []
    for f in facet_strs:
        v0_idx = int(f[1].split("/")[0]) - 1
        v1_idx = int(f[2].split("/")[0]) - 1
        v2_idx = int(f[3].split("/")[0]) - 1

        v0_coord = list(map(lambda x: float(x), vertex_strs[v0_idx][1:]))
        v1_coord = list(map(lambda x: float(x), vertex_strs[v1_idx][1:]))
        v2_coord = list(map(lambda x: float(x), vertex_strs[v2_idx][1:]))
        facets.append(v0_coord + v1_coord + v2_coord)
    return facets


def dispatch_material(
    fn,
    objs: list[list[float]],
    n_material: int,
    has_spec: bool,
    has_lamb: bool,
    has_tran: bool,
    is_light: bool,
    spec_ratio: float,
    lamb_ratio: float,
    tran_ratio: float,
    color: list[float],
    emit:float,
    n: float):
    res:list[str] = list()
    
    for t in objs:
        line = list(map(lambda x: str(x), t))
        line.append(str(n_material))
        line.append("1" if has_spec else "0")
        line.append("1" if has_lamb else "0")
        line.append("1" if has_tran else "0")
        line.append("1" if is_light else "0")
        line.append(str(spec_ratio))
        line.append(str(lamb_ratio))
        line.append(str(tran_ratio))
        line += list(map(lambda x: str(x), color))
        line.append(str(emit))
        line.append(str(n))
        res.append(" ".join(line))
        res.append('\n')
        
    with open(fn,"w") as f:
        f.writelines(res)

dispatch_material("basic_box_blue_wall.obj.mdl", parse_obj("basic_box_blue_wall.obj"), 1, False, True, False, False, 0.0, 1.0, 0.0, [0.0, 0.0, 1.0], 0, 1);
dispatch_material("basic_box_lt.obj.mdl", parse_obj("basic_box_lt.obj"), 1, False,True,False, True, 0.0, 1.0, 0.0, [1.,1.,1.], 600, 1)
dispatch_material("basic_box_spec_wall.obj.mdl", parse_obj("basic_box_spec_wall.obj"), 1, True, False, False, False, 1.0, 0, 0, [1.,1.,1.], 0, 1)
dispatch_material("basic_box_white_walls.obj.mdl", parse_obj("basic_box_white_walls.obj"), 1, False, True, False, False, 0.0, 1.0, 0.0, [1.0,1.0, 1.0], 0, 1)

dispatch_material("basic_box_red_wall.obj.mdl", parse_obj("basic_box_spec_wall.obj"), 1, False, True, False, False, 0.0, 1.0, 0, [1.,0.,0.], 0, 1)
dispatch_material("basic_box_yoi_glass.obj.mdl", parse_obj("basic_box_yoi.obj"), 1, False, False, True, False, 0.0, 0.0, 1.0, [1.,1.,1.], 0, 1.3)

dispatch_material("basic_box_spec_yoi.obj.mdl", parse_obj("basic_box_yoi.obj"), 1, True, False, False, False, 1.0, 0, 0, [1.,1.,1.], 0, 1)

dispatch_material("basic_box_ball_glass.obj.mdl", parse_obj("basic_box_ball.obj"), 1, False, False, True, False, 0.0, 0.0, 1.0, [1.,1.,1.], 0, 1.3)