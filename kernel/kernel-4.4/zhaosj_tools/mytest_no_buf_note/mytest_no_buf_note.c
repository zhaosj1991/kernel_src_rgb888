#include <linux/init.h>
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/string.h>

extern u32 no_buf_count;
extern u64 no_buf_note[100];

static ssize_t no_buf_note_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
   	int i = 0;
    char wait_time_temp[3000];

	sprintf(wait_time_temp, "no_buf_count : %d\n", no_buf_count);
    
    for (i = 0; i < no_buf_count; ++i)
        sprintf(wait_time_temp, "%sno_buf_note[%d] : %lld\n", wait_time_temp, i, no_buf_note[i]);

   	return sprintf(buf, "%s", wait_time_temp);
}

static struct kobj_attribute no_buf_note_attr = __ATTR(no_buf_note_value, 0444, no_buf_note_show, NULL);

/*
 * Create a group of attributes so that we can create and destroy them all
 * at once.
 */
static struct attribute *attrs[] = {
   &no_buf_note_attr.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

/*
 * An unnamed attribute group will put all of the attributes directly in
 * the kobject directory.  If we specify a name, a subdirectory will be
 * created for the attributes with the directory being the name of the
 * attribute group.
 */
static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *helloworld_kobj;

static int __init helloworld_init(void)
{
   int retval;

	/*
	 * Create a simple kobject with the name of "kobject_example",
	 * located under /sys/kernel/
	 *
	 * As this is a simple directory, no uevent will be sent to
	 * userspace.  That is why this function should not be used for
	 * any type of dynamic kobjects, where the name and number are
	 * not known ahead of time.
	 */
	helloworld_kobj = kobject_create_and_add("mytest_notify_sys", kernel_kobj);
	if (!helloworld_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(helloworld_kobj, &attr_group);
	if (retval)
		kobject_put(helloworld_kobj);

	return retval;
}

static void __exit helloworld_exit(void)
{
   kobject_put(helloworld_kobj);
}

module_init(helloworld_init);
module_exit(helloworld_exit);
MODULE_LICENSE("GPL");