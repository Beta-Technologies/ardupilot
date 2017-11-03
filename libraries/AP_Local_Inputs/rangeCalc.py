
expected_low = 1000
expected_high = 2000

def offset_and_ratio(lowest_read, highest_read):
    range_read = highest_read - lowest_read
    expected_range = expected_high - expected_low

    ratio = float(expected_range) / range_read
    offset = -lowest_read 

    return offset, ratio

def apply(offset, ratio, value):
    return (value + offset) * ratio + expected_low

def check_middle(lowest_read, highest_read):
    offset, ratio = offset_and_ratio(lowest_read, highest_read)
    range_read = highest_read - lowest_read
    middle = lowest_read + float(range_read) * 0.5

    adjusted_middle = apply(offset, ratio, middle)
    return adjusted_middle


if __name__ == "__main__":
    lowest_read = 52883
    highest_read = 64415
    offset, ratio = offset_and_ratio(lowest_read, highest_read)
    adjusted_middle = check_middle(lowest_read, highest_read)

    print ("offset: {}, ratio: {}".format(offset, ratio))
    print ("adjusted middle should be 1500: {}".format(adjusted_middle))
