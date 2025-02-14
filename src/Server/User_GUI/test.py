import dlib
import os 
import sys 
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# True가 나오면 GPU가 활성화된 것
print(f"DLIB_USE_CUDA: {dlib.DLIB_USE_CUDA}") 

# True면 GPU 사용 가능
print(f"GPU 사용 가능 여부: {dlib.cuda.get_num_devices() > 0}")  