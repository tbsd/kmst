import sys
from os.path import exists
from PIL import Image, ImageDraw

def graph_to_img(path):
    lines = {}
    with open(path) as fp:
        lines = fp.readlines()
    size = int(lines[0].split()[-1])
    points = [(int(line.split()[0]), int(line.split()[1])) for line in lines[1:]]

    minX = minY = maxX = maxY = 0;
    for p in points:
        if (minX > p[0]):
            minX = p[0]
        if (minY > p[1]):
            minY = p[1]
        if (maxX < p[0]):
            maxX = p[0]
        if (maxY < p[1]):
            maxY = p[1]

    dX = -minX
    dY = -minY

    im = Image.new(mode="RGB", size = (maxX + dX + 1, maxY + dY + 1))

    for p in points:
        im.putpixel((p[0] + dX, p[1] + dY), (255, 255, 255))

    im.save("{}.png".format(path))

def draw_solution(path, edges_path):
    if not exists(edges_path):
        return
    lines = {}
    with open(path) as fp:
        lines = fp.readlines()
    size = int(lines[0].split()[-1])
    points = [(int(line.split()[0]), int(line.split()[1])) for line in lines[1:]]

    minX = minY = maxX = maxY = 0;
    for p in points:
        if (minX > p[0]):
            minX = p[0]
        if (minY > p[1]):
            minY = p[1]
        if (maxX < p[0]):
            maxX = p[0]
        if (maxY < p[1]):
            maxY = p[1]

    dX = -minX
    dY = -minY

    im = Image.new(mode="RGB", size = (maxX + dX + 1, maxY + dY + 1))

    for p in points:
        im.putpixel((p[0] + dX, p[1] + dY), (255, 255, 255))

    lines = {}
    with open(edges_path) as fp:
        lines = fp.readlines()
    edges = [(int(line.split()[1]) - 1, int(line.split()[2]) - 1) for line in lines[2:]]
    draw = ImageDraw.Draw(im)
    for e in edges:
        source = points[e[0]]
        target = points[e[1]]
        edge = [source, target]
        draw.line(edge, fill = "red", width = 0)

    im.save("{}_solution.png".format(path))


graph_to_img("Taxicab_64.txt")
graph_to_img("Taxicab_128.txt")
graph_to_img("Taxicab_512.txt")
graph_to_img("Taxicab_2048.txt")
graph_to_img("Taxicab_4096.txt")

draw_solution("Taxicab_64.txt", "Kurbatov_64.txt") 
draw_solution("Taxicab_128.txt", "Kurbatov_128.txt") 
draw_solution("Taxicab_512.txt", "Kurbatov_512.txt") 
draw_solution("Taxicab_2048.txt", "Kurbatov_2048.txt") 
draw_solution("Taxicab_4096.txt", "Kurbatov_4096.txt") 
