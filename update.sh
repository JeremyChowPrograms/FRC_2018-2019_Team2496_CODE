git add *
echo Type Your Comment:
read myCommit
git commit -m "$myCommit"
git push -u origin master
echo Press enter to continue...
read enterKey