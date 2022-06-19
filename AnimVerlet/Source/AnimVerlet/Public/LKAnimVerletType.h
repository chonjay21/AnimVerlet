#pragma once
#include <CoreMinimal.h>

#if	(ENGINE_MAJOR_VERSION == 5)
	#define LK_UEWIDGET UE::Widget
#else
	#define LK_UEWIDGET FWidget
#endif