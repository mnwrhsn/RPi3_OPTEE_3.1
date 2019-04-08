/*
 * Copyright (c) 2014, STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "testframework.h"

#define MOD_EXP_TEST                                                \
do {                                                                \
	TEE_BigIntConvertFromString(n, nstr);                       \
	nLen = TEE_BigIntFMMContextSizeInU32(nbitsize);             \
	context = (TEE_BigIntFMMContext *)                          \
		TEE_Malloc(nLen * sizeof(TEE_BigIntFMMContext), 0); \
	TEE_BigIntInitFMMContext(context, nLen, n);                 \
	TEE_BigIntConvertFromString(x, xstr);                       \
	TEE_BigIntConvertFromString(e, estr);                       \
	TB_ASSERT_HEX_PRINT_VALUE(dest, resstr);                    \
	TEE_Free(context);                                          \
} while (0)

static void test_modexp(void)
{
	TEE_BigIntFMMContext *context;
	size_t nLen;
	const char *nstr;
	const char *xstr;
	const char *estr;
	const char *resstr;
	int nbitsize;

	TB_INFO("   Testing modular exponentiation");

	DEF_BIGINT(dest, 2048);
	DEF_BIGINT(x, 2048);
	DEF_BIGINT(e, 2048);
	DEF_BIGINT(n, 2048);

	nstr = "2C8D59AF47C81AB3725B472BE417E3BF"
	       "7AB85439AF726ED3DFDF66489D155DC0"
	       "B771C7A50EF7C5E58FB";
	nbitsize = 330;
	xstr = "18EE90FF6C373E0EE4E3F0AD2";
	estr = "AB54A98CEB1F0AD2";

	resstr = "23F455C413C2F037A8A1D06CCFD007BE"
		 "021ABEEE4CE522990A67C65CA94922BD"
		 "822218A79BE6A0AB00C";

	MOD_EXP_TEST;

	nstr = "FFF1FFF1";
	nbitsize = 33;
	xstr = "ABCDEF";
	estr = "1234567";
	resstr = "E9D974C2";
	MOD_EXP_TEST;

	nstr = "C7970CEEDCC3B0754490201A7AA613CD"
		"73911081C790F5F1A8726F463550BB5B"
		"7FF0DB8E1EA1189EC72F93D1650011BD"
		"721AEEACC2ACDE32A04107F0648C2813"
		"A31F5B0B7765FF8B44B4B6FFC93384B6"
		"46EB09C7CF5E8592D40EA33C80039F35"
		"B4F14A04B51F7BFD781BE4D1673164BA"
		"8EB991C2C4D730BBBE35F592BDEF524A"
		"F7E8DAEFD26C66FC02C479AF89D64D37"
		"3F442709439DE66CEB955F3EA37D5159"
		"F6135809F85334B5CB1813ADDC80CD05"
		"609F10AC6A95AD65872C909525BDAD32"
		"BC729592642920F24C61DC5B3C3B7923"
		"E56B16A4D9D373D8721F24A3FC0F1B31"
		"31F55615172866BCCC30F95054C824E7"
		"33A5EB6817F7BC16399D48C6361CC7E5";
	nbitsize = 2048;
	xstr = "63CB86776E61D83AA248100D3D5309E6"
		"B9C88840E3C87AF8D43937A31AA85DAD"
		"BFF86DC70F508C4F6397C9E8B28008DE"
		"B90D775661566F19502083F832461409"
		"D18FAD85BBB2FFC5A25A5B7FE499C25B"
		"237584E3E7AF42C96A07519E4001CF9A"
		"DA78A5025A8FBDFEBC0DF268B398B25D"
		"475CC8E1626B985DDF1AFAC95EF7A925"
		"7BF46D77E936337E01623CD7C4EB269B"
		"9FA21384A1CEF33675CAAF9F51BEA8AC"
		"FB09AC04FC299A5AE58C09D6EE406682"
		"B04F8856354AD6B2C396484A92DED699"
		"5E394AC9321490792630EE2D9E1DBC91"
		"F2B58B526CE9B9EC390F9251FE078D98"
		"98FAAB0A8B94335E66187CA82A641273"
		"99D2F5B40BFBDE0B1CCEA4631B0E63F3";

	estr = "31E5C33BB730EC1D512408069EA984F3"
		"5CE4442071E43D7C6A1C9BD18D542ED6"
		"DFFC36E387A84627B1CBE4F45940046F"
		"5C86BBAB30AB378CA81041FC19230A04"
		"E8C7D6C2DDD97FE2D12D2DBFF24CE12D"
		"91BAC271F3D7A164B503A8CF2000E7CD"
		"6D3C52812D47DEFF5E06F93459CC592E"
		"A3AE6470B135CC2EEF8D7D64AF7BD492"
		"BDFA36BBF49B19BF00B11E6BE275934D"
		"CFD109C250E7799B3AE557CFA8DF5456"
		"7D84D6027E14CD2D72C604EB77203341"
		"5827C42B1AA56B5961CB2425496F6B4C"
		"AF1CA564990A483C93187716CF0EDE48"
		"F95AC5A93674DCF61C87C928FF03C6CC"
		"4C7D558545CA19AF330C3E5415320939"
		"CCE97ADA05FDEF058E6752318D8731FA";

	resstr = "4F7CAEC2D0663A719DC73CF51030906D"
		 "320E9B7AB19AAAF887EFA16E1A92C829"
		 "D7AC7F984E792C94955EFDE31E97E716"
		 "3D9193D3F4A4656EF94C3202DC84D499"
		 "5D25EC33C5284635285639E7B59EB92C"
		 "CA883348AE1F5B78098B0DE87C718CF0"
		 "AA7BE51623A57C1DD8BD1B72E9F0FD17"
		 "98085A5E100810C740C45DB79D391F48"
		 "8BC4ACB3F3C24D5948A994637FF4593B"
		 "7FA8D07F88649EFEED2840EAC47248B6"
		 "5BD3A09A923065A0BDA0EBEB6340AAE1"
		 "08701FEB85811D628467A8D2CCC2D888"
		 "5DE807D0CF65B254B3435E9F844AA86F"
		 "3795ACC2EF71DE64F55EBC638B88B964"
		 "E5C2D55F8F7E3F00B0FC86C19C6AF1EA"
		 "B5CC45E90D7464BA230298F660DDE812";
	MOD_EXP_TEST;

	nstr = "000000022770A7DC599BC90B2FF981CCB5CF05703344C8F3504189ED";
	nbitsize = 2048;
	xstr = "000000020BCFDDA4E89FE86AAC303CB3D767FE66627C797794403831";
	estr = "89DC29F71666F242CBFE60732D73C15C0CD1323CD410627B";
	resstr = "7202419F1301F56B6D0BF9F287D49061F6F378426A78533F";
	MOD_EXP_TEST;

	nstr = "00000002_2770A7DC_599BC90B_2FF981CC_B5CF0570_3344C8F3_50418AAD";
	nbitsize = 2048;
	xstr = "00000001_8D359BB6_4F9B75A9_357C8215_E1E4D5F1_764D016C_F0130738";
	estr = "89DC29F7_1666F242_CBFE6073_2D73C15C_0CD1323C_D41062AB";
	resstr = "1";
	MOD_EXP_TEST;

	DEL_BIGINT(dest);
	DEL_BIGINT(x);
	DEL_BIGINT(e);
	DEL_BIGINT(n);
}

void tb_fmm(void)
{
	const char *TEST_NAME = "Fast Modular Multiplication";

	TB_HEADER(TEST_NAME);
	test_modexp();
	TB_FOOTER(TEST_NAME);
}
