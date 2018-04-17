#ifdef __cplusplus
extern "C" {
#endif

extern int polling_device_register(char *name, char interval, char *callback);
extern int interrupt_device_register(char *name);
extern int interrupt_device_report(char *name, char value);
extern int polling_device_unregister(char *name);
extern int interrupt_device_unregister(char *name);
extern int fault_device_init(void);
extern int fault_device_remove(void);

#ifdef __cplusplus
}
#endif

int fault_device_init(void)
{
    return 1;
}
int polling_device_register(char *name, char interval, char *callback)
{
    return 1;
}
int polling_device_unregister(char *name)
{
    return 1;
}
int fault_device_remove(void)
{
    return 1;
}


