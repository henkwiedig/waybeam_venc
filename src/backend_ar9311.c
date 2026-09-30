#include "backend.h"
#include "ar9311_runtime.h"

const BackendOps *backend_get_selected(void)
{
	return ar9311_runtime_backend_ops();
}
