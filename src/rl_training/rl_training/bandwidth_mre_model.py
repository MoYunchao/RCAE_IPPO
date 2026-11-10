from __future__ import annotations

import math

import torch
import torch.nn as nn


class SingleHeadAttention(nn.Module):
    def __init__(self, embedding_dim: int):
        super().__init__()
        self.key_dim = int(embedding_dim)
        self.norm_factor = 1.0 / math.sqrt(self.key_dim)
        self.tanh_clipping = 10.0
        self.w_query = nn.Parameter(torch.Tensor(embedding_dim, embedding_dim))
        self.w_key = nn.Parameter(torch.Tensor(embedding_dim, embedding_dim))
        self.init_parameters()

    def init_parameters(self) -> None:
        for param in self.parameters():
            nn.init.xavier_uniform_(param)

    def forward(self, q, k, mask=None):
        n_batch, n_key, n_dim = k.size()
        n_query = q.size(1)
        q_flat = q.reshape(-1, n_dim)
        k_flat = k.reshape(-1, n_dim)
        q_proj = torch.matmul(q_flat, self.w_query).view(n_batch, n_query, -1)
        k_proj = torch.matmul(k_flat, self.w_key).view(n_batch, n_key, -1)
        logits = self.norm_factor * torch.matmul(q_proj, k_proj.transpose(1, 2))
        logits = self.tanh_clipping * torch.tanh(logits)
        if mask is not None:
            logits = logits.masked_fill(mask == 1, -1e8)
        return torch.log_softmax(logits, dim=-1)


class MultiHeadAttention(nn.Module):
    def __init__(self, embedding_dim: int, n_heads: int = 8):
        super().__init__()
        self.n_heads = int(n_heads)
        self.embedding_dim = int(embedding_dim)
        self.value_dim = self.embedding_dim // self.n_heads
        self.key_dim = self.value_dim
        self.norm_factor = 1.0 / math.sqrt(self.key_dim)
        self.w_query = nn.Parameter(torch.Tensor(self.n_heads, self.embedding_dim, self.key_dim))
        self.w_key = nn.Parameter(torch.Tensor(self.n_heads, self.embedding_dim, self.key_dim))
        self.w_value = nn.Parameter(torch.Tensor(self.n_heads, self.embedding_dim, self.value_dim))
        self.w_out = nn.Parameter(torch.Tensor(self.n_heads, self.value_dim, self.embedding_dim))
        self.init_parameters()

    def init_parameters(self) -> None:
        for param in self.parameters():
            nn.init.xavier_uniform_(param)

    def forward(self, q, k=None, v=None, key_padding_mask=None, attn_mask=None):
        if k is None:
            k = q
        if v is None:
            v = q

        n_batch, n_key, n_dim = k.size()
        n_query = q.size(1)
        n_value = v.size(1)

        q_flat = q.contiguous().view(-1, n_dim)
        k_flat = k.contiguous().view(-1, n_dim)
        v_flat = v.contiguous().view(-1, n_dim)

        q_proj = torch.matmul(q_flat, self.w_query).view(self.n_heads, n_batch, n_query, -1)
        k_proj = torch.matmul(k_flat, self.w_key).view(self.n_heads, n_batch, n_key, -1)
        v_proj = torch.matmul(v_flat, self.w_value).view(self.n_heads, n_batch, n_value, -1)
        logits = self.norm_factor * torch.matmul(q_proj, k_proj.transpose(2, 3))

        mask = None
        if attn_mask is not None:
            mask = attn_mask.view(1, n_batch, n_query, n_key).expand_as(logits)
        if key_padding_mask is not None:
            key_mask = key_padding_mask.repeat(1, n_query, 1)
            key_mask = key_mask.view(1, n_batch, n_query, n_key).expand_as(logits)
            mask = key_mask if mask is None else mask + key_mask
        if mask is not None:
            logits = logits.masked_fill(mask > 0, -1e8)

        attention = torch.softmax(logits, dim=-1)
        heads = torch.matmul(attention, v_proj)
        out = torch.mm(
            heads.permute(1, 2, 0, 3).reshape(-1, self.n_heads * self.value_dim),
            self.w_out.view(-1, self.embedding_dim),
        ).view(-1, n_query, self.embedding_dim)
        return out, attention


class Normalization(nn.Module):
    def __init__(self, embedding_dim: int):
        super().__init__()
        self.normalizer = nn.LayerNorm(embedding_dim)

    def forward(self, x):
        return self.normalizer(x.view(-1, x.size(-1))).view(*x.size())


class EncoderLayer(nn.Module):
    def __init__(self, embedding_dim: int, n_head: int):
        super().__init__()
        self.attn = MultiHeadAttention(embedding_dim, n_head)
        self.norm1 = Normalization(embedding_dim)
        self.ff = nn.Sequential(nn.Linear(embedding_dim, 512), nn.ReLU(inplace=True), nn.Linear(512, embedding_dim))
        self.norm2 = Normalization(embedding_dim)

    def forward(self, src, key_padding_mask=None, attn_mask=None):
        residual = src
        h = self.norm1(src)
        h, _ = self.attn(q=h, key_padding_mask=key_padding_mask, attn_mask=attn_mask)
        h = h + residual
        residual = h
        h = self.norm2(h)
        h = self.ff(h)
        return h + residual


class DecoderLayer(nn.Module):
    def __init__(self, embedding_dim: int, n_head: int):
        super().__init__()
        self.attn = MultiHeadAttention(embedding_dim, n_head)
        self.norm1 = Normalization(embedding_dim)
        self.ff = nn.Sequential(nn.Linear(embedding_dim, 512), nn.ReLU(inplace=True), nn.Linear(512, embedding_dim))
        self.norm2 = Normalization(embedding_dim)

    def forward(self, tgt, memory, key_padding_mask=None, attn_mask=None):
        residual = tgt
        tgt = self.norm1(tgt)
        memory = self.norm1(memory)
        h, weights = self.attn(q=tgt, k=memory, v=memory, key_padding_mask=key_padding_mask, attn_mask=attn_mask)
        h = h + residual
        residual = h
        h = self.norm2(h)
        h = self.ff(h)
        return h + residual, weights


class Encoder(nn.Module):
    def __init__(self, embedding_dim: int = 128, n_head: int = 8, n_layer: int = 1):
        super().__init__()
        self.layers = nn.ModuleList(EncoderLayer(embedding_dim, n_head) for _ in range(n_layer))

    def forward(self, src, key_padding_mask=None, attn_mask=None):
        for layer in self.layers:
            src = layer(src, key_padding_mask=key_padding_mask, attn_mask=attn_mask)
        return src


class Decoder(nn.Module):
    def __init__(self, embedding_dim: int = 128, n_head: int = 8, n_layer: int = 1):
        super().__init__()
        self.layers = nn.ModuleList(DecoderLayer(embedding_dim, n_head) for _ in range(n_layer))

    def forward(self, tgt, memory, key_padding_mask=None, attn_mask=None):
        weights = None
        for layer in self.layers:
            tgt, weights = layer(tgt, memory, key_padding_mask=key_padding_mask, attn_mask=attn_mask)
        return tgt, weights


class PolicyNet(nn.Module):
    def __init__(self, node_dim: int, embedding_dim: int):
        super().__init__()
        self.node_inputs_embedding = nn.Linear(node_dim, embedding_dim)
        self.graph_node_encoder = Encoder(embedding_dim=embedding_dim, n_head=8, n_layer=6)
        self.self_state_decoder = Decoder(embedding_dim=embedding_dim, n_head=8, n_layer=1)
        self.cooperative_state_decoder = Decoder(embedding_dim=embedding_dim, n_head=8, n_layer=1)
        self.current_embedding = nn.Linear(embedding_dim * 2, embedding_dim)
        self.pointer = SingleHeadAttention(embedding_dim)

    def encode_graph(self, node_inputs, node_padding_mask, edge_mask):
        graph_feature = self.node_inputs_embedding(node_inputs)
        return self.graph_node_encoder(
            src=graph_feature,
            key_padding_mask=node_padding_mask,
            attn_mask=edge_mask,
        )

    def decode_state(self, enhanced_node_feature, current_index, node_padding_mask):
        embedding_dim = enhanced_node_feature.size()[2]
        current_node_feature = torch.gather(enhanced_node_feature, 1, current_index.repeat(1, 1, embedding_dim))
        current_state_feature, _ = self.self_state_decoder(
            current_node_feature,
            enhanced_node_feature,
            node_padding_mask,
        )
        return current_node_feature, current_state_feature

    def get_current_state_feature(self, node_inputs, node_padding_mask, edge_mask, current_index, current_coord):
        del current_coord
        enhanced_node_feature = self.encode_graph(node_inputs, node_padding_mask, edge_mask)
        current_node_feature, current_state_feature = self.decode_state(
            enhanced_node_feature,
            current_index,
            node_padding_mask,
        )
        current_state_feature = self.current_embedding(torch.cat((current_node_feature, current_state_feature), dim=-1))
        return enhanced_node_feature, current_state_feature

    def decode_cooperative_state(self, current_state_feature, msg_stacked):
        enhanced_cooperative_state_feature, _ = self.cooperative_state_decoder(current_state_feature, msg_stacked)
        return enhanced_cooperative_state_feature

    def output_policy(self, enhanced_node_feature, enhanced_cooperative_state_feature, current_edge, edge_padding_mask):
        embedding_dim = enhanced_node_feature.size()[2]
        current_edge_index = current_edge.repeat(1, 1, embedding_dim)
        neighboring_feature = torch.gather(enhanced_node_feature, 1, current_edge_index.to(enhanced_node_feature.device))
        logp = self.pointer(enhanced_cooperative_state_feature, neighboring_feature, edge_padding_mask)
        return logp.squeeze(1)

    def forward(
        self,
        node_inputs,
        node_padding_mask,
        edge_mask,
        current_index,
        current_edge,
        edge_padding_mask,
        current_coord,
        msg_stacked,
    ):
        enhanced_node_feature, current_state_feature = self.get_current_state_feature(
            node_inputs,
            node_padding_mask,
            edge_mask,
            current_index,
            current_coord,
        )
        enhanced_cooperative_state_feature = self.decode_cooperative_state(current_state_feature, msg_stacked)
        return self.output_policy(
            enhanced_node_feature,
            enhanced_cooperative_state_feature,
            current_edge,
            edge_padding_mask,
        )
