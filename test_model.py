"""Exports a test model to ONNX."""

import torch
from torch import Tensor, nn


class TestModule(nn.Module):
    def __init__(self) -> None:
        super().__init__()

        self.a = nn.Linear(10, 10)
        self.b = nn.Linear(10, 10)

    def forward(self, x: Tensor) -> Tensor:
        return self.a(x) + self.b(x)


def save_test_model() -> None:
    model = TestModule()
    jit_model = torch.jit.script(model)
    torch.onnx.export(jit_model, torch.randn(1, 10), "test_model.onnx")


if __name__ == "__main__":
    save_test_model()
