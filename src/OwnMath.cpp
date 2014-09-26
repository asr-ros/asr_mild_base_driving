// this is a -*- C++ -*- file

// -- BEGIN LICENSE BLOCK ----------------------------------------------
//
// You received this file as part of MCA2
// Modular Controller Architecture Version 2
//
// Copyright (C) FZI Forschungszentrum Informatik Karlsruhe
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Klaus Uhl
 * \date    2008-10-30
 *
 */
//----------------------------------------------------------------------
#include "OwnMath.h"

const int tSinCosLookupTable::cTABLE_INDEX_MASK = cTABLE_SIZE-1;
const double tSinCosLookupTable::cSCALE_FACTOR = 16384. / (2. * M_PI);

tSinCosLookupTable sin_cos_lookup_table;
