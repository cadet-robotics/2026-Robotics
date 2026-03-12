build:
    ./gradlew build

deploy:
    ./gradlew deploy

simulate:
    ./gradlew simulate

test:
    ./gradlew test

jar:
    ./gradlew jar

clean:
    ./gradlew clean

build *args:
    ./gradlew build {{args}}

deploy *args:
    ./gradlew deploy {{args}}
