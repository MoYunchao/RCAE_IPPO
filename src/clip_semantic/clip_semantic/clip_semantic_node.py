#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import numpy as np
import cv2
from PIL import Image as PILImage
import torch
import torch.nn.functional as F
import clip
from cv_bridge import CvBridge
import time

# ==================== 精简语义模板 ====================
SLIM_TEMPLATES = {
    "door": [
        "A door with a handle",
        "An open doorway leading to another room",
        "A rectangular door frame seen from inside"
    ],
    "corridor": [
        "A long narrow corridor with walls on both sides",
        "A straight hallway with floor lines",
        "A passageway leading forward"
    ],
    "dead end": [
        "A corridor blocked by a solid wall at the end",
        "A dead end with no passage",
        "A path that ends abruptly with a wall"
    ],
    "water area": [
        "A puddle of water on the floor with reflections",
        "Shiny wet floor area",
        "Blue water accumulation on the ground"
    ]
}
# =====================================================

# ==================== 固定语义价值 ====================
SEMANTIC_VALUES = {
    "door": 0.9,
    "corridor": 0.8,
    "dead end": 0.2,
    "water area": -0.5
}
# =====================================================

# ==================== 语义风险值（0~1，越高越危险） ====================
RISK_VALUES = {
    "door": 0.1,          # 门通常是安全的
    "corridor": 0.2,       # 走廊一般安全
    "dead end": 0.3,       # 死胡同可能需要倒车，有一定风险
    "water area": 0.8      # 积水区危险
}
# ===================================================================


class CLIPSemanticNode(Node):
    def __init__(self):
        super().__init__('clip_semantic_node')

        # ---------- 参数 ----------
        self.declare_parameter('temperature', 0.3)
        self.declare_parameter('semantic_value_scale', 20.0)
        self.declare_parameter('enable_low_confidence_fallback', True)
        self.declare_parameter('confidence_threshold', 0.40)
        self.declare_parameter('unknown_semantic_value', 0.0)
        self.declare_parameter('unknown_semantic_risk', 0.5)
        self.declare_parameter('low_confidence_log_interval_sec', 2.0)

        self.TEMPERATURE = float(self.get_parameter('temperature').value)
        self.semantic_value_scale = float(self.get_parameter('semantic_value_scale').value)
        self.enable_low_confidence_fallback = bool(
            self.get_parameter('enable_low_confidence_fallback').value
        )
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.unknown_semantic_value = float(self.get_parameter('unknown_semantic_value').value)
        self.unknown_semantic_risk = float(self.get_parameter('unknown_semantic_risk').value)
        self.low_confidence_log_interval_sec = max(
            0.0, float(self.get_parameter('low_confidence_log_interval_sec').value)
        )
        self._last_low_conf_log_wall = 0.0

        self.clip_version = "ViT-B/32"          # 可改为 "ViT-L/14" 以获得更高精度
        self.clip_feat_dim = {
            "RN50": 1024,
            "RN101": 512,
            "RN50x4": 640,
            "RN50x16": 768,
            "RN50x64": 1024,
            "ViT-B/32": 512,
            "ViT-B/16": 512,
            "ViT-L/14": 768,
        }[self.clip_version]

        # ---------- ROS2 ----------
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # 发布语义价值分数（原话题）
        self.value_publisher = self.create_publisher(
            Float32,
            'clip_semantic_score',
            10
        )

        # 新增：发布语义风险值
        self.risk_publisher = self.create_publisher(
            Float32,
            'clip_semantic_risk',
            10
        )

        self.bridge = CvBridge()
        self.categories = list(SLIM_TEMPLATES.keys())

        # ---------- CLIP 初始化 ----------
        self.get_logger().info(f"Loading CLIP model {self.clip_version}...")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        if torch.backends.mps.is_available():
            self.device = "mps"
        self.get_logger().info(f"Using device: {self.device}")

        self.clip_model, self.preprocess = clip.load(self.clip_version)
        self.clip_model.to(self.device).eval()

        # ---------- 预计算文本特征 ----------
        self.get_logger().info("Precomputing text features...")
        self.template_feats = {}
        for cat in self.categories:
            self.template_feats[cat] = self.get_text_feats(SLIM_TEMPLATES[cat])

        self.get_logger().info(
            "semantic params: "
            f"temperature={self.TEMPERATURE:.2f}, "
            f"value_scale={self.semantic_value_scale:.2f}, "
            f"low_conf_fallback={self.enable_low_confidence_fallback}, "
            f"threshold={self.confidence_threshold:.2f}, "
            f"unknown_value={self.unknown_semantic_value:.2f}, "
            f"unknown_risk={self.unknown_semantic_risk:.2f}"
        )
        self.get_logger().info("CLIP semantic scoring node ready.")

    def get_img_feats(self, img):
        """提取图像特征（支持多设备）"""
        img_pil = PILImage.fromarray(np.uint8(img))
        img_tensor = self.preprocess(img_pil).unsqueeze(0).to(self.device)
        with torch.no_grad():
            feats = self.clip_model.encode_image(img_tensor).float()
        feats /= feats.norm(dim=-1, keepdim=True)
        return feats.cpu().numpy().astype(np.float32)

    def get_text_feats(self, texts, batch_size=64):
        """批量提取文本特征（支持多设备）"""
        text_tokens = clip.tokenize(texts)
        if self.device == "cuda":
            text_tokens = text_tokens.cuda()
        elif self.device == "mps":
            text_tokens = text_tokens.to("mps")
        else:
            text_tokens = text_tokens.to(self.device)

        n_texts = len(texts)
        feats = np.zeros((n_texts, self.clip_feat_dim), dtype=np.float32)

        start = 0
        while start < n_texts:
            end = min(start + batch_size, n_texts)
            batch_tokens = text_tokens[start:end]
            with torch.no_grad():
                batch_feats = self.clip_model.encode_text(batch_tokens).float()
            batch_feats /= batch_feats.norm(dim=-1, keepdim=True)
            feats[start:end] = batch_feats.cpu().numpy().astype(np.float32)
            start = end

        return feats

    def image_callback(self, msg):
        try:
            # 图像转换
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # 图像特征
            img_feats = self.get_img_feats(rgb_image)      # (1, D)

            # 计算每个类别的最大相似度
            scores = []
            for cat in self.categories:
                cat_feats = self.template_feats[cat]       # (n_templates, D)
                sims = (img_feats @ cat_feats.T).squeeze() # (n_templates,)
                scores.append(np.max(sims))                # 取最大值作为该类别分数
            scores = np.array(scores)

            # softmax 概率
            probs = F.softmax(
                torch.tensor(scores) / self.TEMPERATURE,
                dim=0
            ).numpy()

            best_idx = int(np.argmax(probs))
            best_cat = self.categories[best_idx]
            max_prob = float(probs[best_idx])

            prob_dict = dict(zip(self.categories, probs))
            self.get_logger().info(
                f"Class probabilities: {prob_dict}, best={best_cat}, max_prob={max_prob:.3f}"
            )

            # ---------- 低置信度兜底 ----------
            use_fallback = (
                self.enable_low_confidence_fallback and
                max_prob < self.confidence_threshold
            )
            if use_fallback:
                final_value = self.unknown_semantic_value
                final_risk = self.unknown_semantic_risk
                now = time.monotonic()
                if now - self._last_low_conf_log_wall >= self.low_confidence_log_interval_sec:
                    self._last_low_conf_log_wall = now
                    self.get_logger().warn(
                        f"Low confidence fallback: max_prob={max_prob:.3f} < "
                        f"threshold={self.confidence_threshold:.3f}, "
                        f"use value={final_value:.3f}, risk={final_risk:.3f}"
                    )
            else:
                # ---------- 语义价值分数 ----------
                final_value = 0.0
                for i, cat in enumerate(self.categories):
                    final_value += probs[i] * SEMANTIC_VALUES[cat]
                final_value = final_value * self.semantic_value_scale

                # ---------- 语义风险值 ----------
                final_risk = 0.0
                for i, cat in enumerate(self.categories):
                    final_risk += probs[i] * RISK_VALUES[cat]

            # 发布价值分数
            value_msg = Float32()
            value_msg.data = float(final_value)
            self.value_publisher.publish(value_msg)

            # 发布风险值
            risk_msg = Float32()
            risk_msg.data = float(final_risk)
            self.risk_publisher.publish(risk_msg)

        except Exception as e:
            self.get_logger().error(f"Semantic scoring error: {e}")

    def destroy_node(self):
        self.get_logger().info("Shutting down CLIP Semantic Node")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CLIPSemanticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
