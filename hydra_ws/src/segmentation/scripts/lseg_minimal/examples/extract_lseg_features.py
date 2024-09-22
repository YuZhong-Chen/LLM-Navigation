"""
Extract LSeg features for a given image and display similarity w.r.t. a prompt

NOTE: Will rescale all input images to 640 x 480 resolution
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Union

import clip
import cv2
import matplotlib.pyplot as plt
import torch
import tyro

from lseg import LSegNet


@dataclass
class ProgramArgs:
    checkpoint_path: Union[str, Path] = (
        Path(__file__).parent / "checkpoints" / "lseg_minimal_e200.ckpt"
    )
    backbone: str = "clip_vitl16_384"
    num_features: int = 256
    arch_option: int = 0
    block_depth: int = 0
    activation: str = "lrelu"
    crop_size: int = 480
    query_image: Union[str, Path] = (
        Path(__file__).parent.parent / "images" / "teddybear.jpg"
    )
    prompt: str = "teddy"


if __name__ == "__main__":

    # Process command-line args (if any)
    args = tyro.cli(ProgramArgs)

    # Initialize the model
    net = LSegNet(
        backbone=args.backbone,
        features=args.num_features,
        crop_size=args.crop_size,
        arch_option=args.arch_option,
        block_depth=args.block_depth,
        activation=args.activation,
    )
    # Load pre-trained weights
    net.load_state_dict(torch.load(str(args.checkpoint_path)))
    net.eval()
    net.cuda()

    # Preprocess the text prompt
    clip_text_encoder = net.clip_pretrained.encode_text
    prompt = clip.tokenize(args.prompt)
    prompt = prompt.cuda()

    # Cosine similarity module
    cosine_similarity = torch.nn.CosineSimilarity(dim=1)

    with torch.no_grad():

        # Load the input image
        img = cv2.imread(str(args.query_image))
        print(f"Original image shape: {img.shape}")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (640, 480))
        img = torch.from_numpy(img).float() / 255.0
        img = img[..., :3]  # drop alpha channel, if present
        img = img.cuda()
        img = img.permute(2, 0, 1)  # C, H, W
        img = img.unsqueeze(0)  # 1, C, H, W
        print(f"Image shape: {img.shape}")

        # Extract per-pixel CLIP features (1, 512, H // 2, W // 2)
        img_feat = net.forward(img)
        # Normalize features (per-pixel unit vectors)
        img_feat_norm = torch.nn.functional.normalize(img_feat, dim=1)
        print(f"Extracted CLIP image feat: {img_feat_norm.shape}")

        # Encode the text features to a CLIP embedding
        text_feat = clip_text_encoder(prompt)  # 1, 512
        text_feat_norm = torch.nn.functional.normalize(text_feat, dim=1)
        print(f"Extracted CLIP text feat: {text_feat_norm.shape}")

        # Compute cosine similarity across image and prompt features
        similarity = cosine_similarity(
            img_feat_norm, text_feat_norm.unsqueeze(-1).unsqueeze(-1)
        )
        print("Computed cosine similarity. Displaying...")

        # Visualize similarity
        viz = similarity.detach().cpu().numpy()
        viz = viz[0]
        fig, ax = plt.subplots(1, 2)
        img_to_show = img[0].detach()
        img_to_show = img_to_show.permute(1, 2, 0)
        img_to_show = img_to_show.cpu().numpy()
        ax[0].imshow(img_to_show)
        ax[1].imshow(viz)
        plt.show()
