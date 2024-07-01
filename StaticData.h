/*******************************************************************************
StaticData.h Function definitions for accessing Strawberry String Static Data

Copyright(C) 2024  Howard James May

This file is part of the SweetMaker project

The SweetMaker SDK is free software: you can redistribute it and / or
modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

The SweetMaker SDK is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.If not, see <http://www.gnu.org/licenses/>.

Contact me at sweet.maker@outlook.com
********************************************************************************
Release     Date                        Change Description
--------|-------------|--------------------------------------------------------|
   1      20-Jun-2024   Initial Version
*******************************************************************************/


#ifndef __STATIC_DATA_H__
#define __STATIC_DATA_H__

#include <stdint.h>
#include "StrawberryString.h"
using namespace SweetMaker;

int _setStaticData(StrawberryString::STATIC_DATA* data);
int _getStaticData(StrawberryString::STATIC_DATA* data);

#endif
