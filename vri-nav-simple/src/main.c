#include <stdio.h>
#include <stdlib.h>
#include <ufr.h>

int main() {
    link_t motor = ufr_sys_publisher("motor", "@new zmq:topic @host 127.0.0.1 @port 5002 @coder msgpack");

    ufr_put(&motor, "ii\n", 0, 0);
    while (1) {
        int left, right;
        scanf("%d %d", &left, &right);
        ufr_put(&motor, "ii\n", left, right);
        printf("set motors %d %d\n", left, right);
    }
    return 0;
}