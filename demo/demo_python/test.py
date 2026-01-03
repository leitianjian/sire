import sys
print(sys.path)
try:
    import sire
    print("Success! Path:", sire.__file__)
except ImportError as e:
    print("Error:", e)