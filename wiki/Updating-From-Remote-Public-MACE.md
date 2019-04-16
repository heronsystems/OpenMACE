This document briefly discusses dealing with the remote public repository of MACE. The open-source version of MACE is where much of the core contents, error-reporting, and associated bug fixes relating to the core functionality of MACE shall reside.

## Verify public MACE is a valid remote
The MACE public repository is currently being tracked as a remote branch. This can be verified by opening a command prompt in the top level MACE directory that is linked to GitHub and type
```
$git remote -v
```
which should be accompanied by
```
origin https://github.com/heronsystems/OpenMACE.git(fetch)
origin https://github.com/heronsystems/OpenMACE.git(push)
public https://github.com/heronsystems/MACE.git(fetch)
public https://github.com/heronsystems/MACE.git(push)
```
The remote repository name extension is noted as `public`.

## Fetch the remote branch
Use the `git fetch` command to retrieve new work done within the public branch. Fetching from a repository grabs all the new remote-tracking branches and tags without merging those changes into your own branches. The appropriate syntax for fetching from the remote public MACE branch is
```
$git fetch public
```

## Merge the remote branch
Merging combines the local changes of the current branch with that being merged into from the remote branch.
Traditionally, a developer should not directly merge the master branch from the remote public MACE branch directly into the master branch of this repository. The workflow should be that a developer
1. Creates a new branch,
2. Fetches and merges the remote branch into the working branch,
3. Addresses any merge conflicts
4. Performs V&V of functionality requirements and reasons for pulling in remote
5. Creates a follow on pull request for the current working branch detailing the changes that were merged in from the public repo (link to github notes).

At that time a secondary developer should verify function and accept pull request. The appropriate syntax is
 ```
$git merge remote_name/branch_name
```
where `remote_name` is the remote repository name (in this example `public`) and the `branch_name` is the remote branch the developer desires to pull into the current branch (traditionally this will be `master`)


## Pull from the remote branch
Applying a pull operation is a convenient shortcut for completing both `git fetch` and `git merge` in the same command. Because pull performs a merge on the retrieved changes, the developer should ensure that any local work is committed before running the pull command. If he/she runs into a merge conflict that is unable to be resolved, or if they decide to quit the merge, once could use `git merge --abort` to take the branch back to where it was in before the pull operation. The appropriate syntax is
 ```
$git pull remote_name branch_name
```
where `remote_name` is the remote repository name (in this example `public`) and the `branch_name` is the remote branch the developer desires to pull into the current branch (traditionally this will be `master`)

## Push to the remote branch
This should be an uncommon scenario given how these repositories are expected to be used. If a developer is an active contributor to both repositories and has push access to the public repository, one could push software developments from this repository to the remote. Traditionally, this should be similar to pulling from a remote where the developer pushes back into a branch, regression tests, then generates an appropriate pull request.
 ```
$git push remote_name branch_name
```
where `remote_name` is the remote repository name (in this example `public`) and the `branch_name` is the remote branch the developer desires to push into.
