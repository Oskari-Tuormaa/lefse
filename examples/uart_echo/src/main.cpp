extern int setup_uart();
extern void run_uart();

int main()
{
    if (setup_uart() < 0)
        return 0;
    run_uart();
}
