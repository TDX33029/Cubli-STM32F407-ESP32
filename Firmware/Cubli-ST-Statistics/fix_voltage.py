import sys

filepath = "D:/Document/PrjFile/simpleFOC测试/CubeMX/Core/Src/main.c"

with open(filepath, 'rb') as f:
    data = f.read()

# Add U command for voltage_limit adjustment
old = b"case 'T':   // T6.28"
new = b"case 'U':   // U5.0 - set voltage_limit\n\t\t\t\tvoltage_limit = atof((const char *)(USART_RX_BUF + 1));\n\t\t\t\tprintf(\"voltage_limit=%.4f\\r\\n\", voltage_limit);\n\t\t\t\tbreak;\n\t\t\tcase 'T':   // T6.28"

if old in data:
    data = data.replace(old, new)
    with open(filepath, 'wb') as f:
        f.write(data)
    print("OK - added U command")
else:
    print("NOT FOUND")
    # find 'T' case
    idx = data.find(b"case 'T':")
    if idx >= 0:
        print(repr(data[idx:idx+80]))
