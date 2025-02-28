
    

def reduce_densitiy(input, output, n):
    fi = open(input, "r")
    fo = open(output, "w")
    lines = fi.readlines()

    fo.write(lines[0])
    for i in range(1, len(lines)):
        if (i % n == 0):
            fo.write(lines[i])

INPUT_FILE_PATH = "./inputs/Berlin-1pct-all.csv"
OUTPUT_FILE_PATH = "./inputs/Berlin-1pct-fourth-density.csv"

reduce_densitiy(INPUT_FILE_PATH, OUTPUT_FILE_PATH, 4)