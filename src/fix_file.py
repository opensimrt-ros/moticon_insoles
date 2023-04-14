#!/usr/bin/env python3
import shutil
import sys
import glob
#We need to fix the files that were saved with PN headers but dont have pressure

def fix_file(input_file, output_file):
	#open file
    print("fixing file %s"%input_file)
    replacement_line = "Frame side acc1 acc2 acc3 ang1 ang2 ang3 totalForce cop1 cop2\n"
    with open(input_file) as from_file:
        with open(output_file, "w") as to_file:
            old_beginning = from_file.readline() # and discard
            to_file.write(replacement_line)
            shutil.copyfileobj(from_file, to_file)
            # we just change the first line

def is_fileheader_incorrect(filename):
    a = set()
    header_length = None
    with open(filename, "r") as file:
        for row in file:
            row_length =len(row.split(" ")) 
            if not header_length:
                header_length = row_length 
            else:
                a.add(row_length)
    if not header_length:
        print("file %s is completely empty!!"%filename)
        return False
    if len(a) > 1:
        print("file: %s has variable row lengths %s \nWhile header has row_length %d"%(filename, a, header_length))
        return True
    if len(a) == 0:
        print("file %s is empty (has only a header)!"%filename)
        return False
    if a.pop() != header_length:
        return True
    else: 
        return False

def clean_dir(input_dir):
    for name in glob.glob(input_dir+"/*_insole.txt"):
        # name is an unfixed file
        prefix = name.split("_insole.txt")[0]
        new_name = prefix + "_header_corrected.txt"
        if is_fileheader_incorrect(name):
            fix_file(name, new_name)
        else:
            shutil.copy2(name, new_name)

if __name__ == "__main__":

    if len(sys.argv) == 1:
        print("usage: %s input_file output_file"%sys.argv[0])
        print("usage: %s input_dir"%sys.argv[0])
    elif len(sys.argv) == 2:
        clean_dir(sys.argv[1])
    else:
        fix_file(sys.argv[1], sys.argv[2])
