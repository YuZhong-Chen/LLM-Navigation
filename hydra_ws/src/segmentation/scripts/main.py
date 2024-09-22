#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import clip

import torch
from lseg_minimal.lseg import LSegNet

# Clock
from rosgraph_msgs.msg import Clock

semantics_pub = None
depth_pub = None
rgb_pub = None
current_time = None
model = None
text_feat_norm_list = None
segmentation_classes = None
depth_image = None
cosine_similarity = torch.nn.CosineSimilarity(dim=1)

def prepare(label_input, model_filename):

    model = LSegNet(
        backbone="clip_vitl16_384",
        features=256,
        crop_size=480,
        arch_option=0,
        block_depth=0,
        activation="lrelu",
    )
    model.load_state_dict(torch.load(str(model_filename)))
    model.eval().cuda()

    # Preprocess the text prompt (multiple classes separated by commas)
    clip_text_encoder = model.clip_pretrained.encode_text
    segmentation_classes = label_input

    print(f"Classes of interest: {segmentation_classes}")

    prompt = [clip.tokenize(lc).cuda() for lc in segmentation_classes]
    text_feat_list = [clip_text_encoder(p) for p in prompt]
    text_feat_norm_list = [torch.nn.functional.normalize(tf) for tf in text_feat_list]

    return model, text_feat_norm_list, segmentation_classes

def generate_color_csv(filename):

    # Use get_new_pallete function to generate the colors by class
    pallete = get_new_pallete(len(segmentation_classes))

    with open(filename, "w") as f:
        f.write("name,red,green,blue,alpha,id\n")
        for i, class_name in enumerate(segmentation_classes):
            f.write("{},{},{},{},{},{}\n".format(
                class_name,
                int(pallete[i][0] * 255),
                int(pallete[i][1] * 255),
                int(pallete[i][2] * 255),
                255,
                i
            ))

def generate_label_space(filename):

    with open(filename, "w") as f:
        f.write("---\n")
        f.write("total_semantic_labels: {}\n".format(len(segmentation_classes)))
        f.write("dynamic_labels: []\n")
        f.write("invalid_labels: []\n")
        f.write("object_labels: [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]\n")
        f.write("surface_places_labels: [0, 1, 2, 3]\n")
        f.write("label_names:\n")
        for i, class_name in enumerate(segmentation_classes):
            f.write("- {{label: {}, name: {}}}\n".format(i, class_name))
    
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

def image_callback(msg, semantic_pub, depth_pub, rgb_pub, current_time, p_v):

    if current_time is None:
        return
    global seg_time, depth_image

    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # Use LSegNet to perform semantic segmentation
    with torch.no_grad():

        # Convert the image to tensor
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (640, 480))
        img = torch.from_numpy(img).float() / 255.0
        img = img[..., :3]  # drop alpha channel, if present
        img = img.cuda()
        img = img.permute(2, 0, 1)  # C, H, W
        img = img.unsqueeze(0)  # 1, C, H, W

        # Perform semantic segmentation
        logits = model(img)
        logits_norm = torch.nn.functional.normalize(logits, dim=1)

        # Compute the cosine similarity between the logits and the text features
        similarities = []
        for text_feat_norm in text_feat_norm_list:
            similarity = cosine_similarity(
                logits_norm,
                text_feat_norm.unsqueeze(-1).unsqueeze(-1)
            )
            similarities.append(similarity)

        # Get the class with the highest similarity
        similarities = torch.stack(similarities, dim=0)
        similarities = similarities.squeeze(1)  # num_classes, H // 2, W // 2
        similarities = similarities.unsqueeze(0)  # 1, num_classes, H // 2, W // 2
        class_scores = torch.max(similarities, 1)[1]  # 1, H // 2, W // 2
        class_scores = class_scores[0].detach()

        pallete = get_new_pallete(len(segmentation_classes))

        # Resize class scores to original image size -> int
        class_scores = class_scores.cpu().numpy()
        class_scores = cv2.resize(class_scores, (cv_image.shape[1], cv_image.shape[0]), interpolation=cv2.INTER_NEAREST)

        disp = torch.zeros((class_scores.shape[0], class_scores.shape[1], 3), dtype=torch.float32)
        for i, _ in enumerate(segmentation_classes):
            disp[class_scores == i] = pallete[i]

        # Convert to opencv image
        disp = disp.cpu().numpy()
        disp = (disp * 255).astype(np.uint8)
        # disp = cv2.cvtColor(disp, cv2.COLOR_RGB2BGR)

        # Convert to ROS msgs and publish
        disp_msg = bridge.cv2_to_imgmsg(disp, "rgb8")
        disp_msg.header = msg.header
        # disp_msg.header.stamp = rospy.Time.now()
        # msg.header.stamp = rospy.Time.now()
        disp_msg.header.stamp = current_time

        # print("Message timestamp sec: {}, nsec: {}".format(disp_msg.header.stamp.secs, disp_msg.header.stamp.nsecs))

        semantics_pub.publish(disp_msg)
        
        if depth_image is not None:
            depth_image.header.stamp = rospy.Time.now()
            depth_pub.publish(depth_image)
        rgb_pub.publish(msg)

    if p_v:
        cv2.imshow("Image window", cv_image)
        cv2.imshow("Semantic segmentation", disp)
        cv2.waitKey(1)

def clock_callback(msg):
    
    global current_time
    current_time = msg.clock

def depth_callback(msg, depth_pub):
    global depth_image
    depth_image = msg

def main():

    global semantics_pub, model, text_feat_norm_list, segmentation_classes, depth_pub

    #################################################
    # Init ROS node
    rospy.loginfo("[Semantic Segmentation]: Initializing ROS node...")

    rospy.init_node('semantic_segmentation', anonymous=True)

    #################################################
    # Prepare pur segmentation model
    rospy.loginfo("[Semantic Segmentation]: Preparing the segmentation model...")

    p_v = rospy.get_param("~visualization", default=False)
    p_model_filename = rospy.get_param("~model_filename", default="lseg_minimal_e200.ckpt")
    p_label_input = rospy.get_param("~label_class", default="floor,chair,window,ceiling,light,wall,table,other")
    p_output_csv = rospy.get_param("~output_csv", default=False)
    p_output_csv_filename = rospy.get_param("~output_csv_filename", default="color_mapping.csv")
    p_output_label_space = rospy.get_param("~output_label_space", default=False)
    p_output_label_space_filename = rospy.get_param("~output_label_space_filename", default="label_space.yaml")

    model_filename = p_model_filename
    output_csv_filename = p_output_csv_filename
    label_input = p_label_input.split(",")

    model, text_feat_norm_list, segmentation_classes = prepare(label_input, model_filename)

    if p_output_csv:
        # Output the color mapping csv file
        rospy.loginfo("[Semantic Segmentation]: Generating color mapping csv file to {}".format(output_csv_filename))
        generate_color_csv(output_csv_filename)

    if p_output_label_space:
        # Output the label space yaml file
        rospy.loginfo("[Semantic Segmentation]: Generating label space yaml file to {}".format(p_output_label_space_filename))
        generate_label_space(p_output_label_space_filename)

    #################################################
    # Start ROS related component
    rospy.loginfo("[Semantic Segmentation]: Start ROS related components.")

    semantics_pub = rospy.Publisher("/semantic_segmentation", Image, queue_size=10)
    depth_pub = rospy.Publisher("/depth_segmentation", Image, queue_size=10)
    rgb_pub = rospy.Publisher("/rgb_segmentation", Image, queue_size=10)

    image_cb = lambda msg: image_callback(msg, semantics_pub, depth_pub, rgb_pub, current_time, p_v)
    depth_cb = lambda msg: depth_callback(msg, depth_pub)
    rospy.Subscriber("/zed/zed_node/left_raw/image_raw_color", Image, image_cb)
    rospy.Subscriber("/zed/zed_node/depth/depth_registered", Image, depth_cb)
    rospy.Subscriber("/clock", Clock, clock_callback)

    #################################################
    # Start the spin node
    rospy.loginfo("[Semantic Segmentation]: Start spin node.")
    rospy.spin()

if __name__ == '__main__':
    main()