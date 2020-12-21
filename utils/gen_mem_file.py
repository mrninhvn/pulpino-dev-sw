#!/usr/bin/env python
import sys
WORD=8192
BITS=32
MUX=16
MEM_WIDTH=512
MEM_HEIGHT=512
def get_row_col(addr):
    row = int(addr/MUX)
    col = addr & (MUX-1)
    return (row, col)
def get_bit_vector(data, col=0):
    """ Return data at specific column """
    d = []
    for i in range(0,BITS):
        d.append(((data >> i) & 0x1)<<col)
    return d
def gen_ram_hex(spi_stim, output_file):
    mems = []
    for i in range(0, MEM_HEIGHT):
        mems.append([0 for i in range(0,BITS)])
    with open(spi_stim) as f:
        for l in f:
            addr,data=l.replace("@", "").split(" ")
            row,col = get_row_col(int(addr,16))
            b = get_bit_vector(int(data,16), col)
            old_data = mems[row]
            new_data = [b[i] | old_data[i] for i in range(0,BITS)]
            # b_str = [("%04X" % b[i]) for i in range(0, BITS)]
            # b_str = " ".join(b_str)
            # new_data_str = [("%04X" % new_data[i]) for i in range(0, BITS)]
            mems[row] = new_data
            # print addr, row, col, data, b_str
            # print " ".join(new_data_str)
    with open(output_file, 'w') as f:
        for row in range(0, MEM_HEIGHT):
            row_data = [("%04X" % mems[row][i]) for i in range(BITS-1,-1,-1)]
            f.write("".join(row_data))
            f.write("\n")
        
if __name__ == "__main__":
    gen_ram_hex(sys.argv[1], sys.argv[2])
