```markdown
---
title: Physical AI & Humanoid Robotics Chatbot
emoji: 🤖
colorFrom: blue
colorTo: green
sdk: docker
app_port: 7860
pinned: false
---

# 🤖 Physical AI & Humanoid Robotics Textbook

*A complete, hands-on guide to building autonomous humanoid robots — from Physical AI foundations to full deployment.*

![Status](https://img.shields.io/badge/Status-Complete-success)
![License](https://img.shields.io/badge/License-MIT-blue)
![ROS2](https://img.shields.io/badge/ROS%202-Humble-green)

---

## 📘 Overview

This repository contains a **fully developed technical textbook** on **Physical AI and Humanoid Robotics**, covering the entire pipeline:

**Sensors → ROS 2 → Simulation → GPU Training → Vision-Language-Action → Autonomous Humanoids**

The book is **published as an interactive website**, with an **integrated RAG-based chatbot** that answers questions directly from the book content.

---

## 🧠 What Makes This Book Different

* ✅ **Theory + Practice Balance:** ≈50/50 ratio of concepts and code.
* ✅ **Simulation-First:** Isaac Sim, Unity, and Gazebo tutorials.
* ✅ **Modern Stack:** ROS 2 Humble, VLA models, and RAG.
* ✅ **AI-Powered Support:** Integrated chatbot for instant clarifications.

---

## 📚 Book Structure

### Part I — Foundations & Software
* **Chapter 1-2:** Introduction to Physical AI & Sensor Systems.
* **Chapter 3-4:** ROS 2 Fundamentals & URDF Modeling.

### Part II — Simulation & Ecosystems
* **Chapter 5-6:** Gazebo & Unity Digital Twins.
* **Chapter 7-8:** NVIDIA Isaac Platform & Reinforcement Learning.

### Part III — VLA & Capstone
* **Chapter 9-10:** Vision–Language–Action & Conversational Robotics.
* **Chapter 11-12:** Autonomous Humanoid Capstone Project.

---

## 🖥️ Project Structure
```text
.
├── app/                 # FastAPI Backend Code
│   ├── api/             # API Endpoints (Chat)
│   ├── rag/             # RAG Logic & Retriever
│   └── main.py          # Entry Point
├── content/             # Original textbook source content
├── specs/               # Specifications & planning artifacts
├── Dockerfile           # Deployment configuration
├── requirements.txt     # Python dependencies
└── README.md            # Project documentation

```

---

## 🤖 Integrated RAG Chatbot

The book includes a **Retrieval-Augmented Generation (RAG)** chatbot that:

* Answers questions strictly from textbook content.
* Uses **Qdrant Cloud** for vector search.
* Uses **Groq/OpenAI** for natural language generation.
* Deploys on **Hugging Face Spaces** via Docker.

---

## 🌐 Live Deployment

* **Book Website (Frontend):** Deployed on **Vercel**.
* **Chatbot Backend:** Deployed on **Hugging Face Spaces**.
* **Vector Store:** Qdrant Cloud.
* **Database:** Neon Serverless PostgreSQL.

---

## 🚀 Deployment Instructions

### 1. Backend (Hugging Face)

Ensure the following **Secrets** are set in your Space Settings:

* `GROQ_API_KEY`
* `QDRANT_URL`
* `QDRANT_API_KEY`
* `NEON_DATABASE_URL`

### 2. Frontend (Vercel)

Update the `BACKEND_URL` in your frontend environment variables to point to the Hugging Face Space's **Direct URL**.

---

## 📜 License

**MIT License** - Free for educational, academic, and research use.

---
