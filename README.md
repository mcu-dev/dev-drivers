
# dev-drivers

This project includes submodules that are essential for its functionality. Please follow the guide below to clone or pull this repository correctly.

## Getting Started

### Cloning the Repository with Submodules

When cloning a repository that contains submodules, you must ensure that both the main repository and its submodules are initialized and updated.

#### 1. Clone the Repository

To clone the repository and automatically initialize and update its submodules, use the `--recurse-submodules` flag:

```bash
git clone --recurse-submodules https://github.com/username/project.git
```

#### 2. Navigate to the Project Directory

After cloning, navigate into the project directory:

```bash
cd project
```

#### 3. Initialize and Update Submodules (If Skipped During Clone)

If you have cloned the repository without the `--recurse-submodules` flag, you can manually initialize and update the submodules using the following commands:

```bash
git submodule init
git submodule update
```

Or you can do both steps in one command:

```bash
git submodule update --init --recursive
```

### Pulling Updates with Submodules

When pulling updates from a repository with submodules, you need to ensure the submodules are also updated.

#### 1. Pull the Latest Changes

First, pull the latest changes from the main repository:

```bash
git pull
```

#### 2. Update Submodules

After pulling, update the submodules to ensure they are synced with the main repository:

```bash
git submodule update --recursive
```

Alternatively, you can use this command to pull and update submodules in one step:

```bash
git pull --recurse-submodules
```

### Common Commands

- **Add a new submodule**: 
  ```bash
  git submodule add <repository-url> <path-to-submodule>
  ```

- **Remove a submodule**:
  1. Remove the submodule from the `.gitmodules` file:
     ```bash
     git rm --cached <path-to-submodule>
     ```
  2. Commit the changes and delete the submodule folder:
     ```bash
     rm -rf <path-to-submodule>
     ```

## More Information

For more details on working with submodules, refer to the official [Git documentation on submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules).
