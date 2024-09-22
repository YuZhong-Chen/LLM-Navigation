FROM pytorch/pytorch:2.2.1-cuda12.1-cudnn8-runtime

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    wget \
    unzip \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip install --no-cache-dir \
    numpy \
    pandas \
    matplotlib \
    langchain==0.2.1 \
    langchain-community==0.2.1 \
    langchain-nvidia-ai-endpoints==0.0.18 \
    unstructured==0.14.2 \
    unstructured[pdf]==0.14.2 \
    fastapi==0.109.0 \
    kaleido==0.2.1 \
    uvicorn==0.27.0 \
    sentence-transformers==2.2.2 \
    chromadb==0.4.22

# Set the working directory
WORKDIR /workspace

# Run bash
CMD ["/bin/bash"]