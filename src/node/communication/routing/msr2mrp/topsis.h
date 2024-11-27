//*******************************************************************************
//*  Copyright: Balint Aron Uveges, 2024                                        *
//*  Developed at Pazmany Peter Catholic University,                            *
//*               Faculty of Information Technology and Bionics                 *
//*  Author(s): Balint Aron Uveges                                              *
//*  This file is distributed under the terms in the attached LICENSE file.     *
//*                                                                             *
//*******************************************************************************


#ifndef _TOPSIS_H_
#define _TOPSIS_H_

#include <cmath>
#include <armadillo>

using namespace arma;

struct ps_alt {
    int sink;
    int pathid;
};

struct ps_alt_r : public ps_alt {
    double rank;
};

class TopsisEngine {
    private:
        mat table;
        int alt_num;
        std::vector<ps_alt> alts;
    protected:
        mat normalizeMatrix() {
            mat pow_table = square(table);
            mat norm_table = table;
            rowvec norm(table.n_cols,fill::zeros);
            for(int i=0; i < table.n_cols ; ++i) {
                norm(i)=sum(pow_table.col(i));
            }
            norm=sqrt(norm);
            for(int i=0; i < table.n_rows ; ++i) {
                norm_table.row(i) / norm;
            }
            return norm_table;
        };
    public:
        TopsisEngine(int alt_num): alt_num(alt_num) {
            table=mat(0, 4, fill::zeros);
        };
        void addAlternative(ps_alt id, int hop, double pdr, double nrg, double env) {
            alts.push_back(id);
            rowvec v({static_cast<double>(hop), pdr, nrg, env});
            table.row(table.n_rows)=v;
        };
        std::vector<ps_alt_r> getRanking() {
            auto n_table=normalizeMatrix();

        };
};


#endif // _TOPSIS_H_
