## Some useful helper methods that are needed across all 

# Convert time x (given in nanoseconds) to an output string in microseconds
def from_nano_seconds(x):
    return str(round(x / 1000))

def writeToFile(path, content):
    f = open(path, "w")
    f.write(content)
    f.close()