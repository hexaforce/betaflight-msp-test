# betaflight-msp-test

rm -rf build && mkdir -p build && g++ -o build/msp_reader src/msp_reader.cpp -std=c++11 -Wall -Wextra -O2 -lpthread
rm -rf build && mkdir -p build && g++ -o build/msp_reader src/msp_reader.c -std=c++11 -Wall -Wextra -O2 -lpthread

./build/msp_reader
