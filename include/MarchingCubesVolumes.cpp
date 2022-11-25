#include <cstring>
#include <iostream>
#include <map>
#include <iomanip>

bool ver[8];

void rot_xp(bool* in, bool* out)
{
    out[0] = in[4];
    out[1] = in[5];
    out[2] = in[1];
    out[3] = in[0];
    out[4] = in[7];
    out[5] = in[6];
    out[6] = in[2];
    out[7] = in[3];
};

void rot_xm(bool* in, bool* out)
{
    bool tmp_ver1[8];
    bool tmp_ver2[8];
    rot_xp(in, tmp_ver1);
    rot_xp(tmp_ver1, tmp_ver2);
    rot_xp(tmp_ver2, out);
};

void rot_yp(bool* in, bool* out)
{
    out[0] = in[1];
    out[1] = in[5];
    out[2] = in[6];
    out[3] = in[2];
    out[4] = in[0];
    out[5] = in[4];
    out[6] = in[7];
    out[7] = in[3];
};

void rot_ym(bool* in, bool* out)
{
    bool tmp_ver1[8];
    bool tmp_ver2[8];
    rot_yp(in, tmp_ver1);
    rot_yp(tmp_ver1, tmp_ver2);
    rot_yp(tmp_ver2, out);
};

void rot_zp(bool* in, bool* out)
{
    out[0] = in[3];
    out[1] = in[0];
    out[2] = in[1];
    out[3] = in[2];
    out[4] = in[7];
    out[5] = in[4];
    out[6] = in[5];
    out[7] = in[6];
};

void rot_zm(bool* in, bool* out)
{
    bool tmp_ver1[8];
    bool tmp_ver2[8];
    rot_zp(in, tmp_ver1);
    rot_zp(tmp_ver1, tmp_ver2);
    rot_zp(tmp_ver2, out);
};

void mir_x(bool* in, bool* out)
{
    out[0] = in[1];
    out[1] = in[0];
    out[2] = in[3];
    out[3] = in[2];
    out[4] = in[5];
    out[5] = in[4];
    out[6] = in[7];
    out[7] = in[6];
};

void mir_y(bool* in, bool* out)
{
    out[0] = in[3];
    out[1] = in[2];
    out[2] = in[1];
    out[3] = in[0];
    out[4] = in[7];
    out[5] = in[6];
    out[6] = in[5];
    out[7] = in[4];
};

void mir_z(bool* in, bool* out)
{
    out[0] = in[4];
    out[1] = in[5];
    out[2] = in[6];
    out[3] = in[7];
    out[4] = in[0];
    out[5] = in[1];
    out[6] = in[2];
    out[7] = in[3];
};

uint8_t unq_case[15];
double volume[15];

std::map<uint8_t, double> cube_volume;

enum mir { nm, x, y, z };
enum rot { nr, xp, yp, zp, xm, ym, zm };

int main()
{
    unq_case[0] = 0b00000000;
    unq_case[1] = 0b00000001;
    unq_case[2] = 0b00000011;
    unq_case[3] = 0b00100001;
    unq_case[4] = 0b01000001;
    unq_case[5] = 0b00001110;
    unq_case[6] = 0b01000011;
    unq_case[7] = 0b01010010;
    unq_case[8] = 0b00001111;
    unq_case[9] = 0b10001101;
    unq_case[10] = 0b01010101; //
    unq_case[11] = 0b01001101;
    unq_case[12] = 0b00011110; //
    unq_case[13] = 0b10100101; //
    unq_case[14] = 0b10001110;

    volume[0] = 0;
    volume[1] = 20833333 / 1e9;
    volume[2] = 125000000 / 1e9;
    volume[3] = 2 * volume[1];
    volume[4] = 2 * volume[1];
    volume[5] = 354166666 / 1e9;
    volume[6] = volume[1] + volume[2];
    volume[7] = 3 * volume[1];
    volume[8] = 500000000 / 1e9;
    volume[9] = 500000000 / 1e9;
    volume[10] = 2 * volume[2];
    volume[11] = 500000000 / 1e9;
    volume[12] = volume[1] + volume[5];
    volume[13] = 4 * volume[1];
    volume[14] = 500000000 / 1e9;

    uint8_t tmp_cb;
    bool tmp_bool[8];
    bool tmp_boolm[8];
    bool tmp_boolmr[8];
    bool tmp_boolmrr[8];
    bool tmp_boolmrrr[8];

    for (int i = 0; i < sizeof(unq_case); ++i) {
        tmp_bool[0] = unq_case[i] & (1 << 7);
        tmp_bool[1] = unq_case[i] & (1 << 6);
        tmp_bool[2] = unq_case[i] & (1 << 5);
        tmp_bool[3] = unq_case[i] & (1 << 4);
        tmp_bool[4] = unq_case[i] & (1 << 3);
        tmp_bool[5] = unq_case[i] & (1 << 2);
        tmp_bool[6] = unq_case[i] & (1 << 1);
        tmp_bool[7] = unq_case[i] & (1 << 0);

        //std::cout << unsigned(unq_case[i]) << " - ";
        //std::cout << tmp_bool[0];
        //std::cout << tmp_bool[1];
        //std::cout << tmp_bool[2];
        //std::cout << tmp_bool[3];
        //std::cout << tmp_bool[4];
        //std::cout << tmp_bool[5];
        //std::cout << tmp_bool[6];
        //std::cout << tmp_bool[7] << std::endl;
        for (int miror = mir::nm; miror != z; ++miror) {
            switch (miror) {
            case mir::x:
                mir_x(tmp_boolm, tmp_bool);
                break;
            case mir::y:
                mir_y(tmp_boolm, tmp_bool);
                break;
            case mir::z:
                mir_z(tmp_boolm, tmp_bool);
                break;
            case mir::nm:
                memcpy(tmp_boolm, tmp_bool, sizeof(tmp_bool));
                break;
            }

            for (int rotate1 = rot::nr; rotate1 != zm; ++rotate1) {
                switch (rotate1) {
                case rot::xp:
                    rot_xp(tmp_boolm, tmp_boolmr);
                    break;
                case rot::xm:
                    rot_xm(tmp_boolm, tmp_boolmr);
                    break;
                case rot::yp:
                    rot_yp(tmp_boolm, tmp_boolmr);
                    break;
                case rot::ym:
                    rot_ym(tmp_boolm, tmp_boolmr);
                    break;
                case rot::zp:
                    rot_zp(tmp_boolm, tmp_boolmr);
                    break;
                case rot::zm:
                    rot_zm(tmp_boolm, tmp_boolmr);
                    break;
                case rot::nr:
                    memcpy(tmp_boolmr, tmp_boolm, sizeof(tmp_boolm));
                    break;
                }

                for (int rotate2 = rot::nr; rotate2 != zm; ++rotate2) {
                    switch (rotate2) {
                    case rot::xp:
                        rot_xp(tmp_boolmr, tmp_boolmrr);
                        break;
                    case rot::xm:
                        rot_xm(tmp_boolmr, tmp_boolmrr);
                        break;
                    case rot::yp:
                        rot_yp(tmp_boolmr, tmp_boolmrr);
                        break;
                    case rot::ym:
                        rot_ym(tmp_boolmr, tmp_boolmrr);
                        break;
                    case rot::zp:
                        rot_zp(tmp_boolmr, tmp_boolmrr);
                        break;
                    case rot::zm:
                        rot_zm(tmp_boolmr, tmp_boolmrr);
                        break;
                    case rot::nr:
                        memcpy(tmp_boolmrr, tmp_boolmr, sizeof(tmp_boolmr));
                        break;
                    }

                    for (int rotate3 = rot::nr; rotate3 != zm; ++rotate3) {
                        switch (rotate3) {
                        case rot::xp:
                            rot_xp(tmp_boolmrr, tmp_boolmrrr);
                            break;
                        case rot::xm:
                            rot_xm(tmp_boolmrr, tmp_boolmrrr);
                            break;
                        case rot::yp:
                            rot_yp(tmp_boolmrr, tmp_boolmrrr);
                            break;
                        case rot::ym:
                            rot_ym(tmp_boolmrr, tmp_boolmrrr);
                            break;
                        case rot::zp:
                            rot_zp(tmp_boolmrr, tmp_boolmrrr);
                            break;
                        case rot::zm:
                            rot_zm(tmp_boolmrr, tmp_boolmrrr);
                            break;
                        case rot::nr:
                            memcpy(tmp_boolmrrr, tmp_boolmrr, sizeof(tmp_boolmrr));
                            break;
                        }

                        uint8_t final_cb = 0;
                        final_cb = uint8_t(tmp_boolmrrr[0]) << 7;
                        final_cb += uint8_t(tmp_boolmrrr[1]) << 6;
                        final_cb += uint8_t(tmp_boolmrrr[2]) << 5;
                        final_cb += uint8_t(tmp_boolmrrr[3]) << 4;
                        final_cb += uint8_t(tmp_boolmrrr[4]) << 3;
                        final_cb += uint8_t(tmp_boolmrrr[5]) << 2;
                        final_cb += uint8_t(tmp_boolmrrr[6]) << 1;
                        final_cb += uint8_t(tmp_boolmrrr[7]) << 0;

                        if (cube_volume.count(final_cb)) {
                            if (cube_volume[final_cb] != volume[i]) {
                                std::cout << "Invalid data!" << std::endl;
                                std::cout << unsigned(final_cb) << " - " << cube_volume[final_cb] << " - " << volume[i] << std::endl;
                            }
                        }
                        else {
                            cube_volume[final_cb] = volume[i];
                        }

                        final_cb = 0;
                        final_cb = uint8_t(!tmp_boolmrrr[0]) << 7;
                        final_cb += uint8_t(!tmp_boolmrrr[1]) << 6;
                        final_cb += uint8_t(!tmp_boolmrrr[2]) << 5;
                        final_cb += uint8_t(!tmp_boolmrrr[3]) << 4;
                        final_cb += uint8_t(!tmp_boolmrrr[4]) << 3;
                        final_cb += uint8_t(!tmp_boolmrrr[5]) << 2;
                        final_cb += uint8_t(!tmp_boolmrrr[6]) << 1;
                        final_cb += uint8_t(!tmp_boolmrrr[7]) << 0;

                        double inv_v;
                        switch (i)
                        {
                        case 10:
                        case 12:
                        case 13:
                            inv_v = volume[i];
                            break;
                        default:
                            inv_v = 1.0f - volume[i];
                            break;
                        }

                        if (cube_volume.count(final_cb)) {
                            
                            if (cube_volume[final_cb] != inv_v) {
                                std::cout << "Invalid reverse data!" << std::endl;
                                std::cout << unsigned(final_cb) << " - " << cube_volume[final_cb] << " - " << inv_v << std::endl;

                            }
                        }
                        else {
                            cube_volume[final_cb] = inv_v;
                        }
                    }
                }
            }
        }
    }

    for (uint16_t i = 0; i < 256; ++i)
    {
        // std::cout << "volume[" << unsigned(i) << "] = " << std::fixed << std::setprecision(8) << cube_volume[uint8_t(i)] << ";" << std::endl;
        std::cout << std::fixed << std::setprecision(8) << cube_volume[uint8_t(i)] << ", ";
    }
}

