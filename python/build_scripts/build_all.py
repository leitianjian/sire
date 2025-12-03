import subprocess
from pathlib import Path

sire = Path.cwd().parent.parent.parent
third_parties = sire / 'third_party' / 'install'

scripts = [
    # ['python', 'build_third_parties.py'],
    ['python', 'build.py', '--base-path', str(sire), '--aris-path', str(third_parties / 'aris'), 
     '--fcl-path', str(third_parties / 'hpp-fcl'), '--uuid-path', str(third_parties / 'stduuid'),
     '--toolchain-path', 'D:\\env\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake', '--build-demo', '--build-test', '--build-all']
]

# 依次执行每个Python文件
for script in scripts:
    # 执行脚本并捕获输出
    result = subprocess.run(script, capture_output=True, text=True)
    
    # 打印脚本输出和错误信息
    print(f"Output of {' '.join(script)}:\n{result.stdout}\n{result.stderr}")
    
    # 检查脚本是否成功执行
    if result.returncode != 0:
        print(f"Error executing {' '.join(script)} with return code {result.returncode}")
        break
    else:
        print(f"Successfully executed {' '.join(script)}")
