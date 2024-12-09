#include "circular_buffer.h"
#include <string.h>

void aesd_circular_buffer_init(struct aesd_circular_buffer *buffer) 
{
    memset(buffer, 0, sizeof(struct aesd_circular_buffer));
    for (int i = 0; i < AESDCHAR_MAX_WRITE_OPERATIONS_SUPPORTED; i++) 
    {
        buffer->entry[i].buffptr = buffer->buffer_storage[i];
        buffer->entry[i].size = 0;
    }
}

const char* aesd_circular_buffer_add_entry(struct aesd_circular_buffer *buffer, const struct aesd_buffer_entry *add_entry) 
{
    const char *old_buffer = NULL;

    if (buffer == NULL || add_entry == NULL)
    {
        return NULL;
    }

    if (buffer->full) 
    {
        old_buffer = buffer->entry[buffer->out_offs].buffptr;
        buffer->out_offs = (buffer->out_offs + 1) % AESDCHAR_MAX_WRITE_OPERATIONS_SUPPORTED;
    }

    buffer->entry[buffer->in_offs] = *add_entry;

    buffer->in_offs = (buffer->in_offs + 1) % AESDCHAR_MAX_WRITE_OPERATIONS_SUPPORTED;

    buffer->full = (buffer->in_offs == buffer->out_offs);

    return old_buffer;
}

// Function to read and remove the oldest entry from the circular buffer
const struct aesd_buffer_entry* aesd_circular_buffer_read_and_remove(struct aesd_circular_buffer *buffer) 
{
    if (buffer->in_offs == buffer->out_offs && !buffer->full) 
    {
        return NULL; 
    }

    const struct aesd_buffer_entry* entry = &buffer->entry[buffer->out_offs];
    buffer->out_offs = (buffer->out_offs + 1) % AESDCHAR_MAX_WRITE_OPERATIONS_SUPPORTED;
    buffer->full = false;

    return entry;
}
