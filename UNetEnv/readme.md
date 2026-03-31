## Jetson Setup

### 1. Create a venv on Jetson
```
python3 -m venv ~/unet_env
source ~/unet_env/bin/activate
python --version
```

### 2. Install basic packages
```
sudo apt-get update
sudo apt-get install -y python3-pip libopenblas-dev
pip install --upgrade pip
pip install numpy opencv-python mss pillow matplotlib tqdm
```

### 3. Install PyTorch on Jetson
Do not use the desktop x86 wheel.
On Jetson, you need the Jetson/aarch64 wheel that matches JetPack 5.1.4 / Python 3.8. NVIDIA’s official Jetson PyTorch installation page is the right source of truth for this, and NVIDIA also notes that JetPack 5 on Ubuntu 20.04 is built around Python 3.8 for the provided wheels.

- Open NVIDIA’s Jetson PyTorch install page on Jetson or another browser.
- Pick the wheel for: JetPack 5.x, Python 3.8, aarch64
- Install it inside ~/unet_env

After installing, verify:
```
python -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"
```
