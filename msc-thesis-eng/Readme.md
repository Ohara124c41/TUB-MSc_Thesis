This project is a latex template for a thesis (MSc or BSc).

## Instructions
You can use the Makefile to generate the pdf document:
```
make pdf
```

You can also use the Makefile to clean the auxiliary files created during
the compilation process by latex:
```
make clean
```

## Very important
Once you have cloned this repository, either **disable git** in your local
copy (by removing the .git ordner in your local repository) or **change
the remote server** (origin). Otherwise you will update this repository
with your own content.

### How to change the remote server?
```
git remote set-url origin git://new.url.here
```
Source: [https://stackoverflow.com/questions/2432764/change-the-uri-url-for-a-remote-git-repository]
