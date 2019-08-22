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

extern u32 notify_eof_count;
extern s64 notify_eof[100];

static ssize_t eof_notify_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int i = 0;
    char wait_time_temp[3000];

    wait_time_temp[0] = '\0';
    
    for (i = 0; i < notify_eof_count; ++i)
        sprintf(wait_time_temp, "%s%lld:%lld\n", wait_time_temp, vi_sof_interval[0], notify_eof[i]);

   return sprintf(buf, "%s", wait_time_temp);
}

extern s64 vi_sof_interval_over[100];
extern u32 vi_sof_interval_over_count;
extern s64 vi_sof_interval_over_temp[100];

static ssize_t vi_sof_interval_over_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int i = 0;
    char wait_time_temp[3000];

    wait_time_temp[0] = '\0';
    
    for (i = 0; i < vi_sof_interval_over_count; ++i)
        sprintf(wait_time_temp, "%s%lld :  %lld  :  %lld\n", wait_time_temp, vi_sof_interval[0], 
            vi_sof_interval_over_temp[i],vi_sof_interval_over[i]);

   return sprintf(buf, "%s", wait_time_temp);
}

extern s64 vi_eof_interval_over[100];
extern u32 vi_eof_interval_over_count;

static ssize_t vi_eof_interval_over_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int i = 0;
    char wait_time_temp[3000];

    wait_time_temp[0] = '\0';
    
    for (i = 0; i < vi_eof_interval_over_count; ++i)
        sprintf(wait_time_temp, "%s%lld:%lld\n", wait_time_temp, vi_sof_interval[0], vi_eof_interval_over[i]);

   return sprintf(buf, "%s", wait_time_temp);
}

extern u32 nvhost_syncpt_wait1;
extern u32 nvhost_syncpt_wait2;
extern u32 nvhost_syncpt_wait3;
extern u32 nvhost_syncpt_wait4;
extern u32 nvhost_syncpt_wait5;

extern u32 process_wait_empty_num;
extern u32 process_wait_no_empty_num;

extern u32 completed_waiters_count_0;
extern u32 completed_waiters_count_1;

static ssize_t nvhost_syncpt_wait_count_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
   return sprintf(buf, "%d : %d : %d : %d : %d    empty: %d  no_empty: %d\ncompleted_waiters_count_0 = %d completed_waiters_count_1 = %d\n", 
      nvhost_syncpt_wait1, nvhost_syncpt_wait2, nvhost_syncpt_wait3, nvhost_syncpt_wait4, nvhost_syncpt_wait5
      , process_wait_empty_num, process_wait_no_empty_num, completed_waiters_count_0, completed_waiters_count_1);
}


extern u32 enable_time_thresh;

static ssize_t enable_time_thresh_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
   return sprintf(buf, "%d\n", enable_time_thresh);
}

static ssize_t enable_time_thresh_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
   long int param;
	int ret;

	ret = kstrtol(buf, 10, &param);
   enable_time_thresh = (u32)param;
   if (ret < 0)
		return ret;
   return strlen(buf);;
}

extern s64 sof_enable_time[100];
extern u32 sof_enable_count;

static ssize_t sof_enable_time_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int i = 0;
    char wait_time_temp[3000];

    wait_time_temp[0] = '\0';
    
    for (i = 0; i < sof_enable_count; ++i)
        sprintf(wait_time_temp, "%s%lld\n", wait_time_temp, sof_enable_time[i]);

   return sprintf(buf, "%s", wait_time_temp);
}

extern s64 eof_enable_time[100];
extern u32 eof_enable_count;

static ssize_t eof_enable_time_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int i = 0;
    char wait_time_temp[3000];

    wait_time_temp[0] = '\0';
    
    for (i = 0; i < eof_enable_count; ++i)
        sprintf(wait_time_temp, "%s%lld\n", wait_time_temp, eof_enable_time[i]);

   return sprintf(buf, "%s", wait_time_temp);
}

extern s64 vi_sof_interval_temp[100];
extern u32 vi_sof_interval_temp_count;

static ssize_t vi_sof_interval_temp_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int i = 0;
    char wait_time_temp[3000];

    wait_time_temp[0] = '\0';
    
    for (i = 0; i < vi_sof_interval_temp_count; ++i)
        sprintf(wait_time_temp, "%s%lld\n", wait_time_temp, vi_sof_interval_temp[i]);

   return sprintf(buf, "%s", wait_time_temp);
}


extern u32 waiter_thresh_break_count;
extern int waiter_thresh_break_id[100];
extern u32 waiter_thresh_break_waiter_thresh[100];
extern u32 waiter_thresh_break_sync[100];

static ssize_t waiter_thresh_break_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int i = 0;
    char wait_time_temp[3000];

    wait_time_temp[0] = '\0';
    
    for (i = 0; i < waiter_thresh_break_count; ++i)
        sprintf(wait_time_temp, "%s%d : %d : %d\n", wait_time_temp, 
            waiter_thresh_break_id[i], waiter_thresh_break_waiter_thresh[i], waiter_thresh_break_sync[i]);

   return sprintf(buf, "%s", wait_time_temp);
}


extern u32 frame_count_count;
extern u32 frame_count_FS[100];
extern u32 frame_count_FE[100];
// extern u32 frame_count_BAD[100];

static ssize_t frame_count_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int i = 0;
    char wait_time_temp[3000];

    wait_time_temp[0] = '\0';
    
   //  for (i = 0; i < frame_count_count; ++i)
   //      sprintf(wait_time_temp, "%s FS: %d  FE: %d BAD: %d  SOF_THRESH: %d\n", wait_time_temp, 
   //       frame_count_FS[i], frame_count_FE[i], frame_count_BAD[i], frame_count_THRESH[i]);

   for (i = 0; i < frame_count_count; ++i)
        sprintf(wait_time_temp, "%s  FS: %d  FE: %d\n", wait_time_temp, 
            frame_count_FS[i], frame_count_FE[i]);

   return sprintf(buf, "%s", wait_time_temp);
}

extern u32 syncpt_val_count;
extern u32 syncpt_min_val[100];
extern u32 syncpt_max_val[100];
extern u32 syncpt_thresh_val[100];

static ssize_t syncpt_val_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int i = 0;
    char wait_time_temp[3000];

    wait_time_temp[0] = '\0';
    
    for (i = 0; i < syncpt_val_count; ++i)
        sprintf(wait_time_temp, "%s min: %d  max: %d  thresh: %d\n", wait_time_temp, 
            syncpt_min_val[i], syncpt_max_val[i], syncpt_thresh_val[i]);

   return sprintf(buf, "%s", wait_time_temp);
}

extern u32 sof_val_continue_count;
extern u32 sof_val_continue_old[100];
extern u32 sof_val_continue_new[100];

static ssize_t sof_val_continue_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int i = 0;
    char wait_time_temp[3000];

    wait_time_temp[0] = '\0';
    
    for (i = 0; i < sof_val_continue_count; ++i)
        sprintf(wait_time_temp, "%s old: %d  new: %d\n", wait_time_temp, 
        sof_val_continue_old[i], sof_val_continue_new[i]);

   return sprintf(buf, "%s", wait_time_temp);
}

static struct kobj_attribute rw_thresh_attr = __ATTR(rw_thresh_value, 0664, rw_thresh_show, rw_thresh_store);
static struct kobj_attribute sof_notify_attr = __ATTR(sof_notify_value, 0444, sof_notify_show, NULL);
static struct kobj_attribute eof_notify_attr = __ATTR(eof_notify_value, 0444, eof_notify_show, NULL);
static struct kobj_attribute vi_sof_interval_over_attr = __ATTR(vi_sof_interval_over_value, 0444, vi_sof_interval_over_show, NULL);
static struct kobj_attribute vi_eof_interval_over_attr = __ATTR(vi_eof_interval_over_value, 0444, vi_eof_interval_over_show, NULL);
static struct kobj_attribute nvhost_syncpt_wait_count_attr = __ATTR(nvhost_syncpt_wait_count, 0444, nvhost_syncpt_wait_count_show, NULL);

static struct kobj_attribute enable_time_thresh_attr = __ATTR(enable_time_thresh_value, 0664, enable_time_thresh_show, enable_time_thresh_store);
static struct kobj_attribute sof_enable_time_attr = __ATTR(sof_enable_time_value, 0444, sof_enable_time_show, NULL);
static struct kobj_attribute eof_enable_time_attr = __ATTR(eof_enable_time_value, 0444, eof_enable_time_show, NULL);

static struct kobj_attribute vi_sof_interval_temp_attr = __ATTR(vi_sof_interval_temp_value, 0444, vi_sof_interval_temp_show, NULL);

static struct kobj_attribute waiter_thresh_break_attr = __ATTR(waiter_thresh_break_value, 0444, waiter_thresh_break_show, NULL);
static struct kobj_attribute frame_count_attr = __ATTR(frame_count_value, 0444, frame_count_show, NULL);
static struct kobj_attribute syncpt_val_attr = __ATTR(syncpt_val_value, 0444, syncpt_val_show, NULL);
static struct kobj_attribute sof_val_continue_attr = __ATTR(sof_val_continue_value, 0444, sof_val_continue_show, NULL);

/*
 * Create a group of attributes so that we can create and destroy them all
 * at once.
 */
static struct attribute *attrs[] = {
	&rw_thresh_attr.attr,
	&sof_notify_attr.attr,
   &vi_sof_interval_over_attr.attr,
   &vi_eof_interval_over_attr.attr,
   &nvhost_syncpt_wait_count_attr.attr,
   &enable_time_thresh_attr.attr,
   &sof_enable_time_attr.attr,
   &eof_enable_time_attr.attr,
   &vi_sof_interval_temp_attr.attr,
   &waiter_thresh_break_attr.attr,
   &eof_notify_attr.attr,
   &frame_count_attr.attr,
   &syncpt_val_attr.attr,
   &sof_val_continue_attr.attr,
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