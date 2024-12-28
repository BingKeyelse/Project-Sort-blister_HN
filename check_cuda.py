import torch

# Kiểm tra xem PyTorch có thể sử dụng CUDA không
cuda_available = torch.cuda.is_available()

print(f"CUDA is available: {cuda_available}")

# Nếu có CUDA, in thêm thông tin về thiết bị GPU
if cuda_available:
    print(f"Device name: {torch.cuda.get_device_name(0)}")
    print(f"Device count: {torch.cuda.device_count()}")
else:
    print("CUDA is not available on this system.")
