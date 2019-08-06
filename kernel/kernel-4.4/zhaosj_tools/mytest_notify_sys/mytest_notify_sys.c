#include <linux/init.h>
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/string.h>

extern u32 rw_thresh;

static ssize_t rw_thresh_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
   return sprintf(buf, "%d\n", rw_thresh);
}

static ssize_t rw_thresh_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
   long int param;
	int ret;

	ret = kstrtol(buf, 10, &param);
   rw_thresh = (u32)param;
   if (ret < 0)
		return ret;
   return strlen(buf);;
}

extern u32 notify_sof_count;
extern s64 notify_sof[100];
extern s64 vi_sof_interval[100];

static ssize_t sof_notify_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int i = 0;
    char wait_time_temp[3000];

    wait_time_temp[0] = '\0';
    
    for (i = 0; i < notify_sof_count; ++i)
        sprintf(wait_time_temp, "%s%lld:%lld\n", wait_time_temp, vi_sof_interval[0], notify_sof[i]);

   return sprintf(buf, "%s", wait_time_temp);
}

extern s64 vi_sof_interval_over[100];
extern u32 vi_sof_interval_over_count;

static ssize_t vi_sof_interval_over_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int i = 0;
    char wait_time_temp[3000];

    wait_time_temp[0] = '\0';
    
    for (i = 0; i < vi_sof_interval_over_count; ++i)
        sprintf(wait_time_temp, "%s%lld:%lld\n", wait_time_temp, vi_sof_interval[0], vi_sof_interval_over[i]);

   return sprintf(buf, "%s", wait_time_temp);
}

static struct kobj_attribute rw_thresh_attr = __ATTR(rw_thresh_value, 0664, rw_thresh_show, rw_thresh_store);
static struct kobj_attribute sof_notify_attr = __ATTR(sof_notify_value, 0444, sof_notify_show, NULL);
static struct kobj_attribute vi_sof_interval_over_attr = __ATTR(vi_sof_interval_over_value, 0444, vi_sof_interval_over_show, NULL);

/*
 * Create a group of attributes so that we can create and destroy them all
 * at once.
 */
static struct attribute *attrs[] = {
	&rw_thresh_attr.attr,
	&sof_notify_attr.attr,
   &vi_sof_interval_over_attr.attr,
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