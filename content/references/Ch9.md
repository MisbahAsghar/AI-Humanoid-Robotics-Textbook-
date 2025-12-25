# Chapter 9 References: VLA Pipeline Architecture

## Foundational Sources (Established)

### 1. Brohan et al. (2022) - RT-1
**Citation**: Brohan, A., et al. (2022). "RT-1: Robotics Transformer for Real-World Control at Scale." *arXiv preprint arXiv:2212.06817*.
**URL**: [https://arxiv.org/abs/2212.06817](https://arxiv.org/abs/2212.06817)
**Label**: [established]
**Summary**: RT-1 architecture: vision-language transformer for multi-task robotic control, 130k demonstrations, 97% success on training tasks.
**Relevance**: Core VLA architecture (Section 1.2.1).

### 2. Brohan et al. (2023) - RT-2
**Citation**: Brohan, A., et al. (2023). "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control." *arXiv preprint arXiv:2307.15818*.
**URL**: [https://arxiv.org/abs/2307.15818](https://arxiv.org/abs/2307.15818)
**Label**: [established]
**Summary**: RT-2 co-finetunes VLM (PaLI-X 55B) on web + robot data, 62% novel object success vs RT-1 32%, emergent reasoning.
**Relevance**: Improved VLA with web knowledge transfer (Section 1.2.2).

### 3. Driess et al. (2023) - PaLM-E
**Citation**: Driess, D., et al. (2023). "PaLM-E: An Embodied Multimodal Language Model." *Proc. ICML*, 8469-8488.
**URL**: [https://arxiv.org/abs/2303.03378](https://arxiv.org/abs/2303.03378)
**Label**: [established]
**Summary**: 540B embodied LLM integrating vision, language, sensor data for long-horizon planning and visual QA.
**Relevance**: Large-scale VLA systems (Section 1.2.3).

### 4. Minderer et al. (2022) - OWL-ViT
**Citation**: Minderer, M., et al. (2022). "Simple Open-Vocabulary Object Detection with Vision Transformers." *Proc. ECCV*, 728-755.
**DOI**: [https://doi.org/10.1007/978-3-031-20080-9_42](https://doi.org/10.1007/978-3-031-20080-9_42)
**Label**: [established]
**Summary**: Open-vocabulary detector using CLIP for zero-shot object detection from text queries.
**Relevance**: Vision grounding (Section 1.3.1).

## Tool Documentation

### 5. Hugging Face Transformers (2025)
**Citation**: Wolf, T., et al. (2025). *Transformers: State-of-the-Art NLP*. [Online].
**URL**: [https://huggingface.co/docs/transformers/](https://huggingface.co/docs/transformers/)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: Library for pre-trained models (BERT, GPT, OWL-ViT, PaLI) with inference API.
**Relevance**: VLM implementation (Section 2.2).

### 6. OpenAI API Documentation (2025)
**Citation**: OpenAI. (2025). *OpenAI API Reference*. [Online].
**URL**: [https://platform.openai.com/docs/](https://platform.openai.com/docs/)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: GPT-4, GPT-4V (vision) API for task planning, visual reasoning.
**Relevance**: LLM integration (Section 2.1).

### 7. Ollama Documentation (2025)
**Citation**: Ollama. (2025). *Ollama: Local LLM Inference*. [Online].
**URL**: [https://ollama.ai/](https://ollama.ai/)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: Run Llama 3, Mistral, Phi locally with simple API.
**Relevance**: Local LLM deployment (Section 3.2).

### 8. MoveIt Documentation (2025)
**Citation**: MoveIt 2 Project. (2025). *MoveIt 2 Documentation*. [Online].
**URL**: [https://moveit.ros.org/](https://moveit.ros.org/)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: Motion planning for action primitives.
**Relevance**: Primitive implementation (Section 2.4).

## Emerging Sources

### 9. Llama 3 (2024)
**Citation**: Meta AI. (2024). *Llama 3 Model Card*. [Online].
**URL**: [https://llama.meta.com/llama3/](https://llama.meta.com/llama3/)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: Open-source LLM (8B, 70B parameters) for task planning, comparable to GPT-3.5.
**Relevance**: Local LLM alternative to GPT-4.

### 10. RT-X (2024)
**Citation**: Open X-Embodiment Collaboration. (2024). "Open X-Embodiment: Robotic Learning Datasets and RT-X Models." *arXiv:2310.08864*.
**URL**: [https://arxiv.org/abs/2310.08864](https://arxiv.org/abs/2310.08864)
**Label**: [emerging]
**Summary**: Multi-robot dataset (22 robots, 527k trajectories) for training general VLA policies.
**Relevance**: VLA training data at scale.

### 11. CLIP (2024)
**Citation**: Radford, A., et al. (2021). "Learning Transferable Visual Models From Natural Language Supervision." *Proc. ICML*, 8748-8763.
**URL**: [https://arxiv.org/abs/2103.00020](https://arxiv.org/abs/2103.00020)
**Label**: [established]
**Summary**: Vision-language pre-training enabling zero-shot image classification from text.
**Relevance**: Foundation for open-vocabulary detection.

### 12. Google Gemini (2024)
**Citation**: Google DeepMind. (2024). *Gemini API Documentation*. [Online].
**URL**: [https://ai.google.dev/](https://ai.google.dev/)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: Multimodal LLM (text, images, video) for visual reasoning.
**Relevance**: Alternative to GPT-4V for vision-language tasks.

### 13. Physical Intelligence π₀ (2024)
**Citation**: Physical Intelligence. (2024). *π₀: Foundation Model for Physical Intelligence*. [Online].
**URL**: [https://www.physicalintelligence.company/](https://www.physicalintelligence.company/)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: General-purpose VLA trained on diverse robot data (dexterous manipulation, mobile manipulation, locomotion).
**Relevance**: Latest VLA developments beyond RT-2.

### 14. OpenVLA (2024)
**Citation**: Kim, G., et al. (2024). "OpenVLA: Open-Source Vision-Language-Action Models." *arXiv preprint arXiv:2406.09246*.
**URL**: [https://arxiv.org/abs/2406.09246](https://arxiv.org/abs/2406.09246)
**Label**: [emerging]
**Summary**: Open-source VLA trained on Open X-Embodiment data, 7B parameters, Apache 2.0 license.
**Relevance**: Accessible VLA for research/development.

### 15. Liang et al. (2023) - Code as Policies
**Citation**: Liang, J., et al. (2023). "Code as Policies: Language Model Programs for Embodied Control." *Proc. ICRA*, 9493-9500.
**URL**: [https://arxiv.org/abs/2209.07753](https://arxiv.org/abs/2209.07753)
**Label**: [emerging]
**Summary**: LLM generates Python code for robot control (composing perception + action APIs), enables zero-shot task transfer.
**Relevance**: Alternative VLA approach using code generation.

---

**Note**: All URLs verified December 2025. VLA field rapidly evolving (2022-2024); check for latest models.
