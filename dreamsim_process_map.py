#!/usr/bin/env python

import pandas as pd
import sys
import torch

from dreamsim import dreamsim
from PIL import Image as PILImage
from tqdm import tqdm


image_pose_df = pd.read_csv(sys.argv[1] + "/map.csv",
                             header=None,
                             names=['index', 'x', 'y', 'a', 'filename']
                            )
# init model
torch.hub._validate_not_a_forked_repo=lambda a,b,c: True
model, preprocess = dreamsim(pretrained=True, device="cuda")

for filename in tqdm(image_pose_df['filename']):
    image = PILImage.open(sys.argv[1] + "/" + filename)
    emb = model.embed(preprocess(image).to("cuda"))
    torch.save(emb, filename.split('.')[0] + ".pt")
