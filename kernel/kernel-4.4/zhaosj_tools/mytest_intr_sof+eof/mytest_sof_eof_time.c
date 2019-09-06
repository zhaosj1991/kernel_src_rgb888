#include <linux/init.h>
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/string.h>


extern s64 sof_time_sum;
extern s64 sof_time_count;
extern s64 sof_time_aver;
extern s64 sof_time_max;
extern s64 sof_time_max_index;
extern s64 sof_time_min;

static ssize_t sof_time_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
   return sprintf(buf, " sof_time_sum = %lld\n sof_time_count = %lld\n sof_time_aver = %lld\n sof_time_max = %lld\n sof_time_max_index = %lld\n sof_time_min = %lld\n", 
                  sof_time_sum, sof_time_count, sof_time_aver, sof_time_max, sof_time_max_index, sof_time_min);
}

extern s64 eof_time_sum;
extern s64 eof_time_count;
extern s64 eof_time_aver;
extern s64 eof_time_max;
extern s64 eof_time_max_index;
extern s64 eof_time_min;

static ssize_t eof_time_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
   return sprintf(buf, " eof_time_sum = %lld\n eof_time_count = %lld\n eof_time_aver = %lld\n eof_time_max = %lld\n eof_time_max_index = %lld\n eof_time_min = %lld\n", 
                  eof_time_sum, eof_time_count, eof_time_aver, eof_time_max, eof_time_max_index, eof_time_min);
}

static struct kobj_attribute sof_time_attr = __ATTR(sof_time_value, 0444, sof_time_show, NULL);
static struct kobj_attribute eof_time_attr = __ATTR(eof_time_value, 0444, eof_time_show, NULL);

/*
 * Create a group of attributes so that we can create and destroy them all
 * at once.
 */
static struct attribute *attrs[] = {
   &sof_time_attr.attr,
   &eof_time_attr.attr,
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
	helloworld_kobj = kobject_create_and_add("my_sof_time_value", kernel_kobj);
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