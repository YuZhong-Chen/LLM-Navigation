"""
Extract LSeg features for a given image and display similarity w.r.t. a prompt

NOTE: Will rescale all input images to 640 x 480 resolution
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Union

import clip
import cv2
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import torch
import tyro

from lseg import LSegNet


def get_new_pallete(num_colors):
    """Generate a color pallete given the number of colors needed. First color is always black."""
    pallete = []
    for j in range(num_colors):
        lab = j
        r, g, b = 0, 0, 0
        i = 0
        while lab > 0:
            r |= ((lab >> 0) & 1) << (7 - i)
            g |= ((lab >> 1) & 1) << (7 - i)
            b |= ((lab >> 2) & 1) << (7 - i)
            i = i + 1
            lab >>= 3
        pallete.append([r, g, b])
    return torch.tensor(pallete).float() / 255.0


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
    query_image: Union[str, Path] = Path(__file__).parent.parent / "images" / "cat.jpg"
    segclasses: str = "plant,grass,cat,stone"


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
    # prompts = ["other"]  # begin with the catch-all "other" class
    label_classes = set()
    for _c in args.segclasses.split(","):
        if _c != "other":
            label_classes.add(_c)
    label_classes = list(label_classes)
    label_classes.insert(0, "other")
    print(f"Classes of interest: {label_classes}")
    if len(label_classes) == 1:
        raise ValueError("Need more than 1 class")

    # Cosine similarity module
    cosine_similarity = torch.nn.CosineSimilarity(dim=1)

    with torch.no_grad():

        # Extract and normalize text features
        prompt = [clip.tokenize(lc).cuda() for lc in label_classes]
        text_feat_list = [clip_text_encoder(p) for p in prompt]
        text_feat_norm_list = [
            torch.nn.functional.normalize(tf) for tf in text_feat_list
        ]

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

        # Compute cosine similarity across image and prompt features
        similarities = []
        for _i in range(len(label_classes)):
            similarity = cosine_similarity(
                img_feat_norm, text_feat_norm_list[_i].unsqueeze(-1).unsqueeze(-1)
            )
            similarities.append(similarity)

        similarities = torch.stack(
            similarities, dim=0
        )  # num_classes, 1, H // 2, W // 2
        similarities = similarities.squeeze(1)  # num_classes, H // 2, W // 2
        similarities = similarities.unsqueeze(0)  # 1, num_classes, H // 2, W // 2
        class_scores = torch.max(similarities, 1)[1]  # 1, H // 2, W // 2
        class_scores = class_scores[0].detach()
        print(f"class scores: {class_scores.shape}")

        pallete = get_new_pallete(len(label_classes))

        # img size // 2 for height and width dims
        disp_img = torch.zeros(240, 320, 3)
        for _i in range(len(label_classes)):
            disp_img[class_scores == _i] = pallete[_i]
        rawimg = cv2.imread(str(args.query_image))
        rawimg = cv2.cvtColor(rawimg, cv2.COLOR_BGR2RGB)
        rawimg = cv2.resize(rawimg, (320, 240))
        rawimg = torch.from_numpy(rawimg).float() / 255.0
        rawimg = rawimg[..., :3]  # drop alpha channel, if present

        disp_img = 0.5 * disp_img + 0.5 * rawimg

        plt.imshow(disp_img.detach().cpu().numpy())
        plt.legend(
            handles=[
                mpatches.Patch(
                    color=(
                        pallete[i][0].item(),
                        pallete[i][1].item(),
                        pallete[i][2].item(),
                    ),
                    label=label_classes[i],
                )
                for i in range(len(label_classes))
            ]
        )
        plt.show()
