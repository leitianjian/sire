import subprocess
from pathlib import Path

third_parties = Path.cwd().parent.parent.parent / 'third_party'

scripts = [
    ["python", "build_aris.py", "--base-path", str(third_parties / 'aris'), "--install-dir", str(third_parties / 'install' / 'aris'), "--build-demo", "--build-test", "--build-all"],
    ["python", "build_fcl.py", "--base-path", str(third_parties / 'hpp-fcl'), "--install-dir", str(third_parties / 'install' / 'hpp-fcl'), '--toolchain-path', 'D:\\env\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake', '--build-all'],
    ["python", "build_uuid.py", "--base-path", str(third_parties / 'stduuid'), "--install-dir", str(third_parties / 'install' / 'stduuid'), '--build-all']
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
