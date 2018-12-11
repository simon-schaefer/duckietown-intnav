# /bin/bash

DOCKER_NAME="seleschaefer"
PACKAGE="intnav"
TAG="0.4"

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"
WSPATH="$SCRIPTPATH/../../../"

cp $SCRIPTPATH/.dockerignore $WSPATH
cp $SCRIPTPATH/Dockerfile $WSPATH

cd $WSPATH
docker build -t $DOCKER_NAME/$PACKAGE:$TAG .
docker push $DOCKER_NAME/$PACKAGE:$TAG

rm $WSPATH/.dockerignore
rm $WSPATH/Dockerfile
