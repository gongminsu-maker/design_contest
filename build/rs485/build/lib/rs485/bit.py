low_number = 0b10000000
high_humber = low_number << 8
number = high_humber & low_number
print(bin(low_number))
print(bin(high_humber))
print(bin(number))