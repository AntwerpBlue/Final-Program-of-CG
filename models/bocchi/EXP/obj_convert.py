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

# basic_walls
dispatch_material("white.obj.mdl", parse_obj("white.obj"), 1,
                  False, True, False, False, 0, 1, 0, [0.8, 0.8, 0.8], 0, 1)
dispatch_material("red.obj.mdl", parse_obj("red.obj"), 1,
                  False, True, False, False, 0, 1, 0, [0.8, 0, 0], 0, 1)
dispatch_material("blue.obj.mdl", parse_obj("blue.obj"), 1,
                  False, True, False, False, 0, 1, 0, [0.8, 0, 0], 0, 1)

# mirror
dispatch_material("spec.obj.mdl", parse_obj("spec.obj"), 1,
                  True, False, False, False, 1, 0, 0, [1, 1, 1], 0, 1)

# light
dispatch_material("lt.obj.mdl", parse_obj("lt.obj"), 1,
                  False, True, False, True, 0, 1, 0, [1.0, 0.9, 0.9], 600, 1)

# bocchi
dispatch_material("b_black.obj.mdl", parse_obj("b_black.obj"), 1,
                  False, False, True, False, 0, 0, 1, [0.01, 0.01, 0.01], 0, 1.3)
dispatch_material("b_pink.obj.mdl", parse_obj("b_pink.obj"), 1,
                  False, False, True, False, 0, 0, 1, [236/255, 164/255, 184/255], 0, 1.3)
dispatch_material("b_blue.obj.mdl", parse_obj("b_blue.obj"), 1,
                  False, False, True, False, 0, 0, 1, [118/255, 195/255, 225/255], 0, 1.3)
dispatch_material("b_yellow.obj.mdl", parse_obj("b_yellow.obj"), 1,
                  False, False, True, False, 0, 0, 1, [224/255, 202/255, 90/255], 0, 1.3)
