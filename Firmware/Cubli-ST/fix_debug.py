import sys

filepath = "D:/Document/PrjFile/simpleFOC测试/CubeMX/Core/Src/main.c"

with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
    content = f.read()

# Find the old code block and replace
old_marker = '\t\tif(time1_cntr >= 200)  // '
new_block = '''\t\tif(time1_cntr >= 200)  // 0.2s LED闪烁
\t\t{
\t\t\ttime1_cntr = 0;
\t\t\tHAL_GPIO_TogglePin(GPIOG, GPIO_PIN_0);  // PG0 LED
\t\t\tdebug_timer++;
\t\t\t/* 每1秒打印一次状态 */
\t\t\tif(debug_timer >= 5)
\t\t\t{
\t\t\t\tdebug_timer = 0;
\t\t\t\tprintf("angle=%.3f  CCR1=%lu CCR2=%lu CCR3=%lu\\r\\n",
\t\t\t\t\tshaft_angle,
\t\t\t\t\tTIM1->CCR1, TIM1->CCR2, TIM1->CCR3);
\t\t\t}
\t\t}'''

# Find the exact position
lines = content.split('\n')
for i, line in enumerate(lines):
    if i >= 127 and i <= 135:
        print(f'L{i+1}: {repr(line)}')

# Replace exact block
old_block = '''\t\tif(time1_cntr >= 200)  // 0.2s LED闪烁
\t\t{
\t\t\ttime1_cntr = 0;
\t\t\tHAL_GPIO_TogglePin(GPIOG, GPIO_PIN_0);  // PG0 LED
\t\t}'''

if old_block in content:
    content = content.replace(old_block, new_block)
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(content)
    print("SUCCESS: Added debug serial output")
else:
    print("FAILED: Could not find the old block")
    print(repr(old_block[:80]))
