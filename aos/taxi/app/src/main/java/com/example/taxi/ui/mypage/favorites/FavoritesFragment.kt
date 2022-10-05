package com.example.taxi.ui.mypage.favorites

import android.text.Editable
import android.text.TextWatcher
import android.util.Log
import android.view.View
import androidx.appcompat.widget.SearchView
import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.api.KakaoAPI
import com.example.taxi.data.dto.mypage.Favorites
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.data.dto.user.destination.DestinationSearch
import com.example.taxi.data.dto.user.destination.DestinationSearchDto
import com.example.taxi.databinding.FragmentFavoritesBinding
import com.example.taxi.ui.call_taxi.setting.DestinationSearchListAdapter
import com.example.taxi.ui.home.user.DestinationListAdapter
import com.example.taxi.ui.home.user.FavoritesAdapter
import com.example.taxi.ui.home.user.FavoritesDialogFragment
import com.example.taxi.ui.home.user.UserHomeViewModel
import com.example.taxi.ui.mypage.update_user.UpdateAddressDialogFragment
import com.example.taxi.utils.constant.KakaoApi
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.constant.show
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response
import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory

@AndroidEntryPoint
class FavoritesFragment : BaseFragment<FragmentFavoritesBinding>(R.layout.fragment_favorites) {
    private val userHomeViewModel : UserHomeViewModel by viewModels()

    private lateinit var updateFavoritesAdapter: UpdateFavoritesAdapter
    private var favorites : MutableList<Favorites> = mutableListOf()
    private var addressFavorites = ""
    private lateinit var destinationSearchListAdapter: DestinationSearchListAdapter
    private lateinit var destination : Favorites
    private var checkState = false

    private val destinationSearchClickListener: (View, String, String, String, String) -> Unit = { _, place, address, x, y ->
        destination = Favorites(address,y,place,x)
        checkState = true
        binding.searchFavoritesSearch.setQuery(destination.addressName, false)
        binding.recyclerFavorites.hide()
    }

    // 즐겨찾기 item 삭제
    private val favoritesDeleteClickListener: (View, String) -> Unit = { _, address ->
        addressFavorites = address
        showFavoritesDialog(address)
    }

    override fun init() {
        initView()
        initAdapter()
        setOnClickListeners()
        observerData()
    }

    private fun initView(){
        binding.searchFavoritesSearch.setOnQueryTextListener(object : SearchView.OnQueryTextListener {
            override fun onQueryTextSubmit(query: String?): Boolean {
                // 검색 버튼 누를 때 호출

                return true
            }

            override fun onQueryTextChange(newText: String?): Boolean {
                if (newText != null && newText != "") {
                    if(!checkState){
                        searchKeyword(newText)
                    }else{
                        checkState = false
                    }
                }
                return true
            }
        })
    }

    private fun initAdapter() {
        userHomeViewModel.getFavorites()
        updateFavoritesAdapter = UpdateFavoritesAdapter().apply {
            onItemClickListener = favoritesDeleteClickListener
        }
        binding.recyclerviewFavorites.apply {
            adapter = updateFavoritesAdapter
            layoutManager = LinearLayoutManager(requireContext(), RecyclerView.VERTICAL, false)
        }
    }

    private fun setOnClickListeners(){
        binding.imgFavoritesBack.setOnClickListener{
            requireActivity().onBackPressed()
        }
        binding.searchFavoritesSearch.setOnClickListener {
            binding.recyclerFavorites.show()
        }
        binding.buttonFavoritesUpdate.setOnClickListener{
            if(binding.searchFavoritesSearch.query.toString() != ""){
                favorites.add(destination)
                if(favorites.size == 1){
                    userHomeViewModel.addFavorites(favorites)
                }else{
                    userHomeViewModel.updateFavorites(favorites)
                }
                userHomeViewModel.getFavorites()
            }
        }
    }

    private fun observerData() {
        userHomeViewModel.favorites.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
//                    binding.progressBar.show()
                    binding.textMyPageNoContentFavorites.show()
                    binding.textMyPageFailedFavorites.hide()
                }
                is UiState.Failure -> {
//                    binding.progressBar.hide()
                    binding.textMyPageNoContentFavorites.hide()
                    binding.textMyPageFailedFavorites.show()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                    binding.recyclerviewFavorites.setBackgroundResource(R.drawable.layout_recycler_no_item)
                }
                is UiState.Success -> {
//                    binding.progressBar.hide()
                    binding.textMyPageNoContentFavorites.hide()
                    binding.textMyPageFailedFavorites.hide()
                    favorites = state.data.toMutableList()
                    updateFavoritesAdapter.updateList(favorites)
                    binding.recyclerviewFavorites.setBackgroundResource(R.drawable.layout_recycler)
                }
            }
        }
    }

    // Dialog를 보여줌과 동시에 삭제 명령을 내린다.
    private fun showFavoritesDialog(address: String) {
        FavoritesDialogFragment { favoritesListener(address) }.show(childFragmentManager, "FAVORITES_DIALOG")
    }

    private val favoritesListener: (address: String) -> Unit = {
        if(favorites.size < 2) {
            userHomeViewModel.deleteFavorites()
        }else{
            for(i in 0 until favorites.size){
                if(favorites[i].address == addressFavorites) {
                    favorites.removeAt(i)
                    userHomeViewModel.updateFavorites(favorites)
                }
            }
        }
        userHomeViewModel.getFavorites()
    }

    private fun initSearchAdapter(list : List<DestinationSearch>){
        destinationSearchListAdapter = DestinationSearchListAdapter().apply {
            onItemClickListener = destinationSearchClickListener
        }
        binding.recyclerFavorites.apply {
            adapter = destinationSearchListAdapter
            layoutManager = LinearLayoutManager(requireContext(), RecyclerView.VERTICAL, false)
        }
        destinationSearchListAdapter.updateList(list)
        binding.recyclerFavorites.visibility = View.VISIBLE
    }

    private fun searchKeyword(keyword: String) {
        val retrofit = Retrofit.Builder()   // Retrofit 구성
            .baseUrl(KakaoApi.BASE_URL)
            .addConverterFactory(GsonConverterFactory.create())
            .build()
        val api = retrofit.create(KakaoAPI::class.java)   // 통신 인터페이스를 객체로 생성
        val call = api.getSearchKeyword(KakaoApi.API_KEY, keyword)   // 검색 조건 입력

        // API 서버에 요청
        call.enqueue(object: Callback<DestinationSearchDto> {
            override fun onResponse(
                call: Call<DestinationSearchDto>,
                response: Response<DestinationSearchDto>
            ) {
                //통신 성공 (검색 결과는 response.body()에 담겨있음)
                Log.d("Test", "Raw: ${response.raw()}")
                Log.d("Test", "Body: ${response.body()}")
                if(response.body()!!.documents != null){
                    initSearchAdapter(response.body()!!.documents)
                }
            }

            override fun onFailure(call: Call<DestinationSearchDto>, t: Throwable) {
                // 통신 실패
                Log.w("MainActivity", "통신 실패: ${t.message}")
            }
        })
    }
}