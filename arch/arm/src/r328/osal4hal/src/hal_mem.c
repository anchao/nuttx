#include <hal_mem.h>
#include <nuttx/kmalloc.h>

void *hal_malloc(uint32_t size)
{
	return kmm_malloc(size);
}

void *hal_zalloc(uint32_t size)
{
	return kmm_zalloc(size);
}

void hal_free(void *p)
{
	kmm_free(p);
}

void *hal_align_malloc(uint32_t size, uint32_t align)
{
	void *ptr;
	_uintptr_t wrap_ptr;

	if (align < 4)
		align = 4;
	else if (align & 0x3)
		align &= (~0x3);

	ptr = hal_malloc(size + align);
	if (!ptr)
		return NULL;
	wrap_ptr = (_uintptr_t)(ptr + align);
	wrap_ptr &= (~align);
	/* save actual pointer */
	*((_uintptr_t *)(wrap_ptr - sizeof(_uintptr_t))) = (_uintptr_t)ptr;
	return (void *)wrap_ptr;
}

void hal_align_free(void *p)
{
	void *ptr;
	if (!p)
		return;
	/* get actual pointer */
	ptr = (void *)(*(_uintptr_t *)(p - sizeof(_uintptr_t)));
	free(ptr);
}
