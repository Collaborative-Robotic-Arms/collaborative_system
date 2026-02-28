# Collaborative_system

This repository uses **Git Submodules** to manage multiple internal components and dependencies. To ensure all submodules are downloaded correctly, please follow the installation steps below.

## 🛠 Installation & Setup

These instructions assume you are setting up a standard ROS 2 style workspace.

### 1. Clone the Repository

Use the `--recursive` flag to automatically initialize and update every submodule in the repository.

```bash
# Create and enter the source directory
mkdir -p collab_ws/src
cd collab_ws/src

# Clone the repository and all submodules
git clone --recurse-submodules https://github.com/Collaborative-Robotic-Arms/collaborative_system.git .

```

### 2. Build the Workspace

Once cloned, navigate back to the workspace root to build the project.

```bash
cd ..
colcon build

```

---

## 💡 Quick Tips for Submodules

If you've already cloned the repo without the `--recursive` flag, or if someone else updates a submodule, use these commands:

* **To initialize submodules after cloning:**
```bash
git submodule update --init --recursive

```


* **To pull the latest changes for all submodules:**
```bash
git submodule update --recursive --remote

```
