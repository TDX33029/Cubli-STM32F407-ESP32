filepath = "D:/Document/PrjFile/simpleFOC测试/CubeMX/Core/Src/main.c"

with open(filepath, 'rb') as f:
    data = f.read()

# Fix duplicate U case - find the first occurrence of two U cases
old1 = b"case 'U':   // U5.0 - set voltage_limit\n\t\t\t\tvoltage_limit = atof((const char *)(USART_RX_BUF + 1));\n\t\t\t\tprintf(\"voltage_limit=%.4f\\r\\n\", voltage_limit);\n\t\t\t\tbreak;\n\t\t\tcase 'U':   // U5.0 - set voltage_limit\n\t\t\t\tvoltage_limit = atof((const char *)(USART_RX_BUF + 1));\n\t\t\t\tprintf(\"voltage_limit=%.4f\\r\\n\", voltage_limit);\n\t\t\t\tbreak;\n\t\t\tcase 'T':   // T6.28"
new1 = b"case 'U':   // U5.0 - set voltage_limit\n\t\t\t\tvoltage_limit = atof((const char *)(USART_RX_BUF + 1));\n\t\t\t\tprintf(\"voltage_limit=%.4f\\r\\n\", voltage_limit);\n\t\t\t\tbreak;\n\t\t\tcase 'T':   // T6.28"

if old1 in data:
    data = data.replace(old1, new1)
    with open(filepath, 'wb') as f:
        f.write(data)
    print("Fixed duplicate U case")
else:
    print("Pattern not found, checking file content...")
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()
    # Find all case 'U' occurrences
    import re
    for m in re.finditer(r"case 'U'.*", content):
        print(f"  Found at pos {m.start()}: {repr(m.group())}")
