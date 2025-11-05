from dreamsim import dreamsim
import torch

device = "cuda" if torch.cuda.is_available() else "cpu"
# This will download the pretrained model into the cache
model, preprocess = dreamsim(pretrained=True, device=device)
print("Model downloaded successfully!")