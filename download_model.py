from dreamsim import dreamsim
import torch

device = "cuda" if torch.cuda.is_available() else "cpu"

model, preprocess = dreamsim(pretrained=True, device=device)
print("Model downloaded successfully!")