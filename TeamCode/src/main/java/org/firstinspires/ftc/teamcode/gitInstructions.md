# Git Usage Notes

General git commands and commit message format for the team. Provides info to pull from Pedro repo and switch branches per season/comp

---

## Repo Setup (one time, per machine)

After cloning the repo, add Pedro's original repo as a second remote so you can pull their updates later. This only needs to be done once per machine:

```bash
git remote add upstream https://github.com/Pedro-Pathing/Quickstart.git
```

Check your remotes are set up right:
```bash
git remote -v
```
Should show `origin` (our repo) and `upstream` (Pedro's repo).

---

## Commit Message Format

Format: `[day] [type of day] [tag]: short desc`

Example: `D1 comp fix: driver 2 offset step changes`

### Day
Meet counter, allows to easily navigate changes and roll back if needed

### Type of Day
- **comp** - competition day, stuff changed/fixed at an actual comp
- **meet** - regular meeting, normal day
- **scrim** - scrimmage day, meet with other teams

### Tag (what kind of change)
- **test** - not final code, creating something new to test later
- **feat** - new feature, added something that wasnt there before (new subsystem, new paths, new commands, etc.)
- **fix** - bug fix, dealing with something thats not working/broke
- **chore** - cleanup, refactor, renaming stuff, no actual "behavior" change
- **docs** - new md or txt file, readme updates, comments

### Examples
```
D6 meet feat: added slew rate
D7 scrim fix: updated turret offset in auto
D41 comp chore: moves old autos to trash
D67 meet docs: added more got commands 
```

---
## Template Updates

Use this workflow when making global template changes, update base configurations, or fix something directly on `main` so that every other branch can inherit the update.

`main` is our template branch. It currently mirrors Pedro's `pedro3` branch since Pedro hasn't merged that into their `master` yet. Once they do, we'll pull from `upstream/master` instead — check with whoever manages the repo if unsure which one is current.

1. Switch to main and pull the absolute latest from Pedro:
```bash
git checkout main
git fetch upstream
git merge upstream/pedro3
```
(swap `upstream/pedro3` for `upstream/master` once Pedro merges their beta stuff in)

2. Make edits directly to the files.

3. Commit the template changes using the team format:
```bash
git add .
git commit -m "D[X] meet chore: updated template configurations"
```

4. Push the changes to the team's remote repository:
```bash
git push origin main
```

5. Copy the updates down to your development branches (repeat this for `Offseason-2026`, `Biobuzz`, etc.):
```bash
git checkout Offseason-2026
git merge main
git push origin Offseason-2026
```

---

## Branch Switching (checkout)

Switch to existing branch, this is when you already have the branch and just wanna hop onto it:
```bash
git checkout branch-name
```

Make new branch and switch to it, use this when you need a brand new branch that doesnt exist yet:
```bash
git checkout -b new-branch-name
```

See what branches exist, shows a list so you can double check the name before switching:
```bash
git branch
```

---

## Pulling Updates From Pedro Repo

Still relevant when we want fresh pedro code without losing our stuff.

1. Stash your work, this saves your uncommitted changes so they dont get in the way:
```bash
git stash
```

2. Update the tracker branch (this branch just mirrors whatever pedro has):
```bash
git checkout main
git fetch upstream
git merge upstream/pedro3
```

If pedro merged pedro3 into master instead, do this:
```bash
git fetch upstream
git checkout main
git merge upstream/master
```

3. Go back to your actual branch and merge main in:
```bash
git checkout Offseason-2026
git merge main
```
(swap Offseason-2026 for Biobuzz or whatever branch)

4. Bring your stashed work back:
```bash
git stash pop
```

### If conflicts happen
Android Studio's merge tool can get stuck (sometimes only leaves an Abort button). If that happens, resolve from terminal instead:

To take our own changes and ignore theirs:
```bash
git checkout --ours .
git add .
git commit -m "merge conflict resolved, kept ours"
```

To take theirs and overwrite ours (use this when pulling a Pedro update and we just want their version):
```bash
git checkout --theirs .
git add .
git commit -m "merge conflict resolved, kept theirs"
```

If you just want to bail out of the merge completely and go back to before you started:
```bash
git merge --abort
```

---

## Daily Essentials

Check whats changed / whats staged, run this before you do basically anything else so you know where you stand:
```bash
git status
```

Stage stuff to commit, this is everything in the current folder and below:
```bash
git add .
```
or a specific file, safer if you only want certain changes going in:
```bash
git add filename
```

Actually commit, the message goes in quotes after -m:
```bash
git commit -m "mon meet fix: x was fixed"
```

Push to remote, sends your commits up to github:
```bash
git push
```

Pull latest from remote, grabs whatever teammates pushed and merges into your current branch:
```bash
git pull
```

Save uncommitted work without committing (temporary save, useful before switching branches):
```bash
git stash
```
bring it back later, applies the most recent stash and removes it from the stash list:
```bash
git stash pop
```

See commit history, --oneline keeps it to one line per commit so its easier to scan:
```bash
git log --oneline
```
 
---

## Less Common but Useful

See what actually changed before committing, shows line by line diffs:
```bash
git diff
```
just staged changes, same thing but only for whats already added:
```bash
git diff --staged
```

Undo changes to a file (careful, this deletes edits and cant be undone):
```bash
git checkout -- filename
```

Undo a commit safely (makes a new commit that reverses it, keeps history, good for stuff already pushed):
```bash
git revert commit-hash
```

Undo a commit but keep changes staged (use this if you commited too early or with wrong message but the actual code changes are still good, moves your branch pointer back one commit while everything you changed stays staged, ready to re-commit):
```bash
git reset --soft HEAD~1
```

Undo a commit AND unstage the changes (changes stay in your files, just not staged anymore, use this if you want to review/edit files more before staging again):
```bash
git reset HEAD~1
```

Undo a commit and throw away the changes completely (careful, this deletes your work, only use if you're sure you dont want those changes at all anymore):
```bash
git reset --hard HEAD~1
```

Force overwrite a local branch to exactly match a remote branch (careful, throws away any local commits that arent on the remote):
```bash
git reset --hard upstream/pedro3
```

Merge one branch into the current one, brings in all the commits from other-branch:
```bash
git merge other-branch
```

Remove a tracked file from git (and optionally folder), this deletes it from disk too:
```bash
git rm filename
```
remove from git but keep the file locally, good if you accidentally committed something like a build folder:
```bash
git rm --cached filename
```