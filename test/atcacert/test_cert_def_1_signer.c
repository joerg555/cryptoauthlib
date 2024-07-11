#include "atca_test.h"
#ifndef DO_NOT_TEST_CERT

#include "atcacert/atcacert_def.h"

const uint8_t g_test_signer_1_ca_public_key[64] = {
    0x44, 0xCE, 0xAE, 0x5E, 0x80, 0x2E, 0xE7, 0x16, 0x9D, 0x77, 0xDB, 0x0A, 0x55, 0x5A, 0x38, 0xED,
    0xB2, 0x88, 0xAC, 0x73, 0x61, 0x56, 0xCA, 0x5B, 0x20, 0x0B, 0x57, 0x94, 0x7A, 0x48, 0x63, 0x50,
    0xE9, 0x72, 0xC4, 0x11, 0x3D, 0x71, 0x9A, 0xAF, 0x83, 0x72, 0x0E, 0xEF, 0x94, 0x3B, 0xDA, 0x69,
    0xD8, 0x39, 0x20, 0xD5, 0x23, 0xB8, 0x1C, 0x96, 0x49, 0x7C, 0x26, 0x62, 0x00, 0x3B, 0x7C, 0x01
};

const uint8_t g_test_cert_template_1_signer[] = {
    0x30, 0x82, 0x01, 0xB1, 0x30, 0x82, 0x01, 0x57, 0xA0, 0x03, 0x02, 0x01, 0x02, 0x02, 0x03, 0x40,
    0x01, 0x02, 0x30, 0x0A, 0x06, 0x08, 0x2A, 0x86, 0x48, 0xCE, 0x3D, 0x04, 0x03, 0x02, 0x30, 0x36,
    0x31, 0x10, 0x30, 0x0E, 0x06, 0x03, 0x55, 0x04, 0x0A, 0x0C, 0x07, 0x45, 0x78, 0x61, 0x6D, 0x70,
    0x6C, 0x65, 0x31, 0x22, 0x30, 0x20, 0x06, 0x03, 0x55, 0x04, 0x03, 0x0C, 0x19, 0x45, 0x78, 0x61,
    0x6D, 0x70, 0x6C, 0x65, 0x20, 0x41, 0x54, 0x45, 0x43, 0x43, 0x35, 0x30, 0x38, 0x41, 0x20, 0x52,
    0x6F, 0x6F, 0x74, 0x20, 0x43, 0x41, 0x30, 0x1E, 0x17, 0x0D, 0x31, 0x35, 0x30, 0x37, 0x33, 0x31,
    0x30, 0x30, 0x31, 0x32, 0x31, 0x35, 0x5A, 0x17, 0x0D, 0x33, 0x35, 0x30, 0x37, 0x33, 0x31, 0x30,
    0x30, 0x31, 0x32, 0x31, 0x35, 0x5A, 0x30, 0x3A, 0x31, 0x10, 0x30, 0x0E, 0x06, 0x03, 0x55, 0x04,
    0x0A, 0x0C, 0x07, 0x45, 0x78, 0x61, 0x6D, 0x70, 0x6C, 0x65, 0x31, 0x26, 0x30, 0x24, 0x06, 0x03,
    0x55, 0x04, 0x03, 0x0C, 0x1D, 0x45, 0x78, 0x61, 0x6D, 0x70, 0x6C, 0x65, 0x20, 0x41, 0x54, 0x45,
    0x43, 0x43, 0x35, 0x30, 0x38, 0x41, 0x20, 0x53, 0x69, 0x67, 0x6E, 0x65, 0x72, 0x20, 0x58, 0x58,
    0x58, 0x58, 0x30, 0x59, 0x30, 0x13, 0x06, 0x07, 0x2A, 0x86, 0x48, 0xCE, 0x3D, 0x02, 0x01, 0x06,
    0x08, 0x2A, 0x86, 0x48, 0xCE, 0x3D, 0x03, 0x01, 0x07, 0x03, 0x42, 0x00, 0x04, 0xF8, 0x0D, 0x8B,
    0x65, 0xE8, 0xBC, 0xCE, 0x14, 0x76, 0xE1, 0x8D, 0x05, 0xE2, 0x01, 0x69, 0x3B, 0xA2, 0xA6, 0x59,
    0xCF, 0xB9, 0xFD, 0x95, 0xE7, 0xBA, 0xD0, 0x21, 0x77, 0xF1, 0x38, 0x76, 0x1B, 0x34, 0xF1, 0xB3,
    0x58, 0x95, 0xA1, 0x35, 0x0D, 0x94, 0x82, 0x47, 0xE5, 0x23, 0x6F, 0xB3, 0x92, 0x01, 0x51, 0xD1,
    0x3A, 0x6F, 0x01, 0x23, 0xD6, 0x70, 0xB5, 0xE5, 0x0C, 0xE0, 0xFF, 0x49, 0x31, 0xA3, 0x50, 0x30,
    0x4E, 0x30, 0x0C, 0x06, 0x03, 0x55, 0x1D, 0x13, 0x04, 0x05, 0x30, 0x03, 0x01, 0x01, 0xFF, 0x30,
    0x1D, 0x06, 0x03, 0x55, 0x1D, 0x0E, 0x04, 0x16, 0x04, 0x14, 0x1F, 0xAF, 0x8F, 0x90, 0x86, 0x5F,
    0x7D, 0xD2, 0x26, 0xB0, 0x6F, 0xE3, 0x20, 0x4E, 0x48, 0xA5, 0xD2, 0x94, 0x65, 0xE2, 0x30, 0x1F,
    0x06, 0x03, 0x55, 0x1D, 0x23, 0x04, 0x18, 0x30, 0x16, 0x80, 0x14, 0x77, 0x23, 0xA2, 0xC4, 0x32,
    0xA6, 0x94, 0x1D, 0x81, 0x32, 0xCB, 0x76, 0x04, 0xC3, 0x80, 0x1D, 0xD2, 0xBE, 0x95, 0x5D, 0x30,
    0x0A, 0x06, 0x08, 0x2A, 0x86, 0x48, 0xCE, 0x3D, 0x04, 0x03, 0x02, 0x03, 0x48, 0x00, 0x30, 0x45,
    0x02, 0x20, 0x43, 0x90, 0xCD, 0x89, 0xE0, 0x75, 0xD0, 0x45, 0x93, 0x7B, 0x37, 0x3F, 0x52, 0x6F,
    0xF6, 0x5C, 0x4B, 0x4C, 0xCA, 0x7C, 0x61, 0x3C, 0x5F, 0x9C, 0xF2, 0xF4, 0xC9, 0xE7, 0xCE, 0xDF,
    0x24, 0xAA, 0x02, 0x21, 0x00, 0x89, 0x52, 0x36, 0xF3, 0xC3, 0x7C, 0xD7, 0x9D, 0x5C, 0x43, 0xF4,
    0xA9, 0x1B, 0xB3, 0xB1, 0xC7, 0x3E, 0xB2, 0x66, 0x74, 0x6C, 0x20, 0x53, 0x0A, 0x3B, 0x90, 0x77,
    0x6C, 0xA9, 0xC7, 0x79, 0x0D
};

const atcacert_def_t g_test_cert_def_1_signer = {
    .type                       = CERTTYPE_X509,
    .template_id                = 1,
    .chain_id                   = 0,
    .private_key_slot           = 0,
    .sn_source                  = SNSRC_SIGNER_ID,
    .cert_sn_dev_loc            = {
        .zone                   = DEVZONE_NONE,
        .slot                   = 0,
        .is_genkey              = 0,
        .offset                 = 0,
        .count                  = 0
    },
    .issue_date_format          = DATEFMT_RFC5280_UTC,
    .expire_date_format         = DATEFMT_RFC5280_UTC,
    .tbs_cert_loc               = {
        .offset                 = 4,
        .count                  = 347
    },
    .expire_years               = 28,
    .public_key_dev_loc         = {
        .zone                   = DEVZONE_DATA,
        .slot                   = 11,
        .is_genkey              = 0,
        .offset                 = 0,
        .count                  = 72
    },
    .comp_cert_dev_loc          = {
        .zone                   = DEVZONE_DATA,
        .slot                   = 12,
        .is_genkey              = 0,
        .offset                 = 0,
        .count                  = 72
    },
    .std_cert_elements          = {
        {   // STDCERT_PUBLIC_KEY
            .offset             = 205,
            .count              = 64
        },
        {   // STDCERT_SIGNATURE
            .offset             = 363,
            .count              = 73
        },
        {   // STDCERT_ISSUE_DATE
            .offset             = 90,
            .count              = 13
        },
        {   // STDCERT_EXPIRE_DATE
            .offset             = 105,
            .count              = 13
        },
        {   // STDCERT_SIGNER_ID
            .offset             = 174,
            .count              = 4
        },
        {   // STDCERT_CERT_SN
            .offset             = 15,
            .count              = 3
        },
        {   // STDCERT_AUTH_KEY_ID
            .offset             = 331,
            .count              = 20
        },
        {   // STDCERT_SUBJ_KEY_ID
            .offset             = 298,
            .count              = 20
        }
    },
    .cert_elements              = NULL,
    .cert_elements_count        = 0,
    .cert_template              = g_test_cert_template_1_signer,
    .cert_template_size         = sizeof(g_test_cert_template_1_signer)
};
#endif
