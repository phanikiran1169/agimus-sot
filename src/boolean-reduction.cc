// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of sot_hpp.
// sot_hpp is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// sot_hpp is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// sot_hpp. If not, see <http://www.gnu.org/licenses/>.

#include <dynamic-graph/factory.h>

#include <sot/core/binary-op.hh>

#include <sot/hpp/config.hh>

namespace dg = ::dynamicgraph;

namespace dynamicgraph {
  namespace sot {
    namespace hpp {

// -- Start -- copy pasted from sot-core/src/matrix/operators.cpp ------------//

    template< typename TypeRef >
    struct TypeNameHelper
    {
      static const std::string typeName;
    };
    template< typename TypeRef >
    const std::string TypeNameHelper<TypeRef>::typeName = "unspecified";

#define ADD_KNOWN_TYPE( typeid ) \
    template<>const std::string TypeNameHelper<typeid>::typeName = #typeid

    ADD_KNOWN_TYPE(double);
    ADD_KNOWN_TYPE(Vector);
    ADD_KNOWN_TYPE(Matrix);
    ADD_KNOWN_TYPE(MatrixRotation);
    ADD_KNOWN_TYPE(MatrixTwist);
    ADD_KNOWN_TYPE(MatrixHomogeneous);
    ADD_KNOWN_TYPE(VectorQuaternion);
    ADD_KNOWN_TYPE(VectorRollPitchYaw);

      template< typename TypeIn1,typename TypeIn2, typename TypeOut >
        struct BinaryOpHeader
        {
          typedef TypeIn1 Tin1;
          typedef TypeIn2 Tin2;
          typedef TypeOut Tout;
          static const std::string & nameTypeIn1(void) { return TypeNameHelper<Tin1>::typeName; }
          static const std::string & nameTypeIn2(void) { return TypeNameHelper<Tin2>::typeName; }
          static const std::string & nameTypeOut(void) { return TypeNameHelper<Tout>::typeName; }
          void addSpecificCommands(Entity&, Entity::CommandMap_t& ) {}
          virtual std::string getDocString () const
          {
            return std::string
              ("Undocumented binary operator\n"
               "  - input  ") + nameTypeIn1 () +
              std::string ("\n"
                  "  -        ") + nameTypeIn2 () +
              std::string ("\n"
                  "  - output ") + nameTypeOut () +
              std::string ("\n");
          }
        };

#define REGISTER_BINARY_OP( OpType,name )				\
  template<>								\
  const std::string BinaryOp< OpType >::CLASS_NAME = std::string(#name); \
  Entity *regFunction##_##name( const std::string& objname )		\
  {									\
    return new BinaryOp< OpType >( objname );				\
  }									\
  EntityRegisterer regObj##_##name( std::string(#name),&regFunction##_##name)

// -- End ---- copy pasted from sot-core/src/matrix/operators.cpp ------------//

      template < typename T >
        struct Comparison : public BinaryOpHeader <T, T, bool>
      {
        void operator()( const T& a,const T& b, bool& res ) const
        {
          res = ( a < b);
        }
        virtual std::string getDocString () const
        {
          return std::string
            ("Comparison of inputs:\n"
             "  - input  ") + BinaryOpHeader<T,T,T>::nameTypeIn1 () +
            std::string ("\n"
                "  -        ") + BinaryOpHeader<T,T,T>::nameTypeIn2 () +
            std::string ("\n"
                "  - output ") + BinaryOpHeader<T,T,T>::nameTypeOut () +
            std::string ("\n""  sout = ( sin1 < sin2 )\n");
        }
      };

      REGISTER_BINARY_OP (Comparison<double>, CompareDouble);
    } // namespace hpp
  } // namespace sot
} // namespace dynamicgraph
