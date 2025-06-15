import torch
import torch.nn as nn

class TransformerModel(nn.Module):
    def _init_(self, vocab_size, embedding_dim, max_seq_len):
        super(TransformerModel, self)._init_()
        self.embedding = nn.Embedding(vocab_size, embedding_dim)
        self.pos_encoder = nn.Parameter(torch.zeros(1, max_seq_len, embedding_dim))
        self.transformer = nn.Transformer(d_model=embedding_dim, nhead=8)
        self.fc = nn.Linear(embedding_dim, vocab_size)
    
    def forward(self, src):
        src = self.embedding(src) + self.pos_encoder[:, :src.size(1), :]
        output = self.transformer(src, src)
        output = self.fc(output)
        return output

def load_model(model_path, vocab_size, embedding_dim, max_seq_len):
    model = TransformerModel(vocab_size, embedding_dim, max_seq_len)
    model.load_state_dict(torch.load(model_path))
    model.eval()
    return model
