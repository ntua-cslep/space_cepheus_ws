# Daily Git Workflow (Short & Practical)

This is the **minimal, repeatable workflow** you’ll use day‑to‑day. No theory, no ceremony.

---

## 1. New day → start from `main`
Make sure you start clean.

```bash
git checkout main
git pull origin main
```

---

## 2. Create a new feature branch
Name it after what you’re doing.

```bash
git checkout -b feature/<short-description>
```

Examples:
- `feature/docker-cleanup`
- `feature/vicon-sync-fix`
- `feature/thesis-logging`

---

## 3. Work normally
Edit code, configs, scripts, docs.

Check status anytime:
```bash
git status
```

---

## 4. Staging workflow (what goes into a commit)

### A. See exactly what changed
```bash
git status
```

Detailed diff:
```bash
git diff
```

---

### B. Stage everything (fast, early development)
Use when:
- You’re early in the feature
- Changes are tightly related

```bash
git add .
```

---

### C. Stage specific files (recommended default)
Use when:
- Multiple things changed
- You want clean, focused commits

```bash
git add path/to/file.cpp
git add docker/Dockerfile
```

---

### D. Stage parts of files (surgical mode)
Use when:
- One file contains unrelated changes
- You want future‑you to stay sane

```bash
git add -p
```

Keys:
- `y` = stage this hunk
- `n` = skip
- `s` = split hunk
- `q` = quit

---

### E. Unstage if you staged too much

```bash
git restore --staged <file>
```

Or everything:
```bash
git restore --staged .
```

---

## 5. Commit your work (as often as needed)

```bash
git commit -m "Describe what changed"
```

Examples:
- `Add initial docker setup`
- `Fix vicon frame alignment`
- `WIP: refactor control node`

Rule of thumb:
> One commit = one logical change

---

## 6. Push your branch to GitHub

First push:
```bash
git push -u origin feature/<short-description>
```

Next pushes:
```bash
git push
```

Repeat **work → stage → commit → push** as many times as needed.

---

## 7. Finish work → open a Pull Request (GitHub)

On GitHub:
1. Select **Compare & pull request**
2. Base: `main`
3. Compare: your feature branch
4. Add title + short description
5. Create Pull Request

---

## 8. Merge the Pull Request
When ready:
- Merge into `main`
- Choose **Squash** or **Merge commit** (either is fine)
- Delete the feature branch after merge (recommended)

---

## 9. Rollbacks & Recovery (When Things Go Sideways)

### A. Undo local changes (not committed yet)
Discard changes in tracked files:
```bash
git restore .
```

Unstage files (keep edits):
```bash
git restore --staged .
```

---

### B. Undo the last commit (local only)
Keep changes, just remove the commit:
```bash
git reset --soft HEAD~1
```

Throw away the commit *and* changes (be careful):
```bash
git reset --hard HEAD~1
```

---

### C. Revert a commit that is already pushed
Safe option (creates a new commit that undoes the change):
```bash
git revert <commit-hash>
```

Use this when:
- The commit is already on GitHub
- You don’t want to rewrite history

---

### D. Your branch is a mess → start over
Delete the local branch:
```bash
git checkout main
git branch -D feature/<short-description>
```

Recreate it cleanly:
```bash
git checkout -b feature/<short-description>
```

---

### E. Panic button (read‑only inspection)
See recent commits:
```bash
git log --oneline --decorate -10
```

Nothing is truly lost unless you force‑push recklessly.

---

## 10. Ready for the next task

```bash
git checkout main
git pull origin main
```

Start again at step 2.

---

## Mental model (remember this)

```
main = stable history
feature branches = disposable workspaces
staging = intent
commits = checkpoints
pull request = safety gate
```

Mistakes are normal. Git is a time machine. Use it.

